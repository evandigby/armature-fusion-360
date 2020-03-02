#include "JointPlate.h"

#include "UI.h"

namespace ArmatureJoint {
	shared_ptr<JointPlate> JointPlate::create(Ptr<Component> _component, Ptr<ConstructionPlane> _plane, shared_ptr<Values> _values, bool _top) {
		auto plate = shared_ptr<JointPlate>(new JointPlate(_component, _plane, _values, _top));
		if (!plate)
			return nullptr;

		auto body = plate->body();
		if (!body)
			return nullptr;

		return plate;
	}

	Ptr<BRepBody> JointPlate::body() {
		if (_body)
			return _body;

		auto sketch = plateSketch();
		if (!sketch)
			return nullptr;

		auto extrude = plateExtrude();
		if (!extrude)
			return nullptr;

		auto bodies = extrude->bodies();

		if (bodies->count() != 1)
			return nullptr;

		auto body = bodies->item(0);
		if (!body)
			return nullptr;

		body->name("Plate");

		if (!plateChamfer(body))
			return nullptr;

		if (!plateFillet(body))
			return nullptr;

		_body = body;

		return _body;
	}

	Ptr<Sketch> JointPlate::plateSketch() {
		if (_sketch)
			return _sketch;

		auto sketches = component->sketches();
		if (!sketches)
			return nullptr;

		auto sketch = sketches->add(plane);
		if (!sketch)
			return nullptr;

		if (!sketch->name("Joint Plate"))
			return nullptr;

		auto curves = sketch->sketchCurves();
		auto lines = curves->sketchLines();

		auto rectangle = lines->addTwoPointRectangle(Point3D::create(0, 0, 0), Point3D::create(values->length(), -values->width(), 0));
		if (!rectangle)
			return nullptr;

		auto circles = curves->sketchCircles();

		double expectedSubtractionArea = 0;

		auto boltCircle = circles->addByCenterRadius(
			Point3D::create(
				values->length() / 2,
				-values->width() / 2,
				0
			),
			values->boltHoleRadius()
		);
		if (!boltCircle)
			return nullptr;

		for (auto row = 1; row <= values->rows(); row++) {
			for (auto col = 1; col <= values->cols(); col++) {
				auto jointType = values->jointType(row, col);

				if (jointType != ARMATURE_JOINT_OPTION_BALL)
					continue;

				auto ballCircle = circles->addByCenterRadius(
					Point3D::create(
						values->ballX(col),
						values->ballY(row),
						0
					),
					values->circleRadius()
				);
				if (!ballCircle)
					return nullptr;
			}
		}

		_sketch = sketch;

		return _sketch;
	}

	Ptr<ExtrudeFeature> JointPlate::plateExtrude() {
		if (_extrude)
			return _extrude;

		auto profiles = plateSketch()->profiles();
		if (!profiles || profiles->count() < 1)
			return nullptr;

		auto expectedArea = values->expectedArea();
		auto delta = expectedArea * 0.005; // Low calculation accuracy is within 0.5%. We don't have many profiles so that's fine.

		Ptr<Profile> profile;
		for (auto i = 0; i < profiles->count(); i++)
		{
			auto current = profiles->item(i);
			if (!current)
				return nullptr;

			auto areaProps = current->areaProperties(CalculationAccuracy::LowCalculationAccuracy);
			if (!areaProps)
				return nullptr;

			auto area = areaProps->area();
			if (area > expectedArea - delta && area < expectedArea + delta) {
				profile = current;
				break;
			}
		}
		if (!profile)
			return nullptr;

		auto thicknessInput = ValueInput::createByReal(values->thickness());
		if (!thicknessInput)
			return nullptr;

		_extrude = component->
			features()->
			extrudeFeatures()->
			addSimple(profile, thicknessInput, FeatureOperations::NewBodyFeatureOperation);

		return _extrude;
	}

	bool JointPlate::plateChamfer(Ptr<BRepBody> plateBody) {
		auto chamferEdges = ObjectCollection::create();

		auto circ = values->circleCircumference();
		auto delta = circ * 0.005;

		auto edges = plateBody->edges();
		if (!edges)
			return false;

		auto geo = plane->geometry();
		if (!geo)
			return false;

		auto normal = geo->normal();
		auto origin = geo->origin();

		for (auto j = 0; j < edges->count(); j++) {
			auto edge = edges->item(j);
			if (!edge)
				return false;

			if (edge->length() > circ + delta || edge->length() < circ - delta)
				continue;

			auto p = edge->pointOnEdge();
			if (!p)
				return false;

			auto end = p->copy();
			if (!end)
				return false;

			if (!end->translateBy(normal))
				return false;

			auto line = Line3D::create(origin, p);
			if (!line)
				return false;

			auto onPlane = geo->isParallelToLine(line);

			if ((onPlane && top) || (!onPlane && !top))
				chamferEdges->add(edge);
		}

		auto chamfers = component->features()->chamferFeatures();

		auto chamferInput = chamfers->createInput(chamferEdges, false);
		if (!chamferInput)
			return false;

		if (!chamferInput->setToDistanceAndAngle(
			ValueInput::createByReal(values->chamferLength()),
			ValueInput::createByReal(values->chamferAngle())
		))
			return false;

		auto count = chamferEdges->count();
		if (count > 0) {
			auto chamfer = chamfers->add(chamferInput);
			if (!chamfer)
				return false;
		}

		return true;
	}

	bool JointPlate::plateFillet(Ptr<BRepBody> plateBody) {
		auto filletEdges = ObjectCollection::create();

		auto lengthDelta = values->thickness() * 0.005;

		auto edges = plateBody->edges();
		if (!edges)
			return false;

		for (auto j = 0; j < edges->count(); j++) {
			auto edge = edges->item(j);

			if (!edge)
				return false;

			if (edge->length() < values->thickness() + lengthDelta && edge->length() > values->thickness() - lengthDelta) {
				filletEdges->add(edge);
				continue;
			}
		}

		auto fillets = component->features()->filletFeatures();
		if (!fillets)
			return false;

		auto filletInput = fillets->createInput();
		if (!filletInput)
			return false;

		if (!filletInput->addConstantRadiusEdgeSet(filletEdges, ValueInput::createByReal(values->ballRadius()), false))
			return false;

		auto fillet = fillets->add(filletInput);
		if (!fillet)
			return false;

		return true;
	}
};