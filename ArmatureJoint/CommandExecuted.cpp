#include "CommandExecuted.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <string>

#include "JointPlate.h"
#include "UI.h"

namespace ArmatureJoint {
	bool CommandExecuted::createJointNuts(Ptr<Component> component, shared_ptr<Values> values) {
		auto planes = component->constructionPlanes();
		if (!planes)
			return false;

		for (auto col = 1; col <= values->cols(); col++) {
			auto planeInput = planes->createInput(component->xZConstructionPlane());
			if (!planeInput)
				return false;

			planeInput->setByOffset(component->yZConstructionPlane(), ValueInput::createByReal(values->ballX(col)));

			auto plane = planes->add(planeInput);
			if (!plane)
				return false;

			plane->name("Joint Nuts " + std::to_string(col));

			auto sketches = component->sketches();
			if (!sketches)
				return false;

			auto sketch = sketches->add(plane);
			if (!sketch)
				return false;

			sketch->name("Joint Nuts Sketch" + std::to_string(col));

			auto added = 0;

			auto curves = sketch->sketchCurves();
			if (!curves)
				return false;

			auto lines = curves->sketchLines();
			if (!lines)
				return false;

			auto circles = curves->sketchCircles();
			if (!circles)
				return false;

			auto thirty = (30.0 / 180.0) * M_PI; // half of the hex angle
			for (auto row = 1; row <= values->rows(); row++) {
				auto jointType = values->jointType(row, col);

				if (jointType != ARMATURE_JOINT_OPTION_NUT)
					continue;

				added++;

				auto off = values->ballOffset();

				auto centre = Point3D::create(values->ballY(row), values->ballZ(), 0);
				if (!centre)
					return false;

				auto topCentre = Point3D::create(centre->x(),  centre->y() + off, 0);
				if (!topCentre)
					return false;

				// opp = adj / tan(theta)
				auto opp = values->ballOffset() * tan(thirty);

				std::list<Ptr<Point3D>> hexPoints;

				hexPoints.push_back(Point3D::create(centre->x() - opp, centre->y() + off, 0));
				hexPoints.push_back(Point3D::create(centre->x() + opp, centre->y() + off, 0));
				hexPoints.push_back(Point3D::create(centre->x() + (2 * opp), centre->y(), 0));
				hexPoints.push_back(Point3D::create(centre->x() + opp, centre->y() - off, 0));
				hexPoints.push_back(Point3D::create(centre->x() - opp, centre->y() - off, 0));
				hexPoints.push_back(Point3D::create(centre->x() - (2 * opp), centre->y(), 0));

				Ptr<Point3D> last = nullptr;
				Ptr<Point3D> first = nullptr;
				
				for (auto p = hexPoints.begin(); p != hexPoints.end(); p++) {
					if (last != nullptr) {
						auto line = lines->addByTwoPoints(last, *p);
						if (!line)
							return false;
					}
					else {
						first = *p;
					}
					last = *p;
				}
				auto line = lines->addByTwoPoints(last, first);
				if (!line)
					return false;

				auto circle = circles->addByCenterRadius(centre, values->boltHoleRadius());
				if (!circle)
					return false;
			}

			if (added == 0) {
				sketch->deleteMe();
				plane->deleteMe();
				return true;
			}

			auto features = component->features();
			if (!features)
				return false;

			auto extrudes = features->extrudeFeatures();
			if (!extrudes)
				return false;

			auto profiles = sketch->profiles();
			if (!profiles)
				return false;

			auto boltArea = values->boltCircleArea();
			auto boltDelta = boltArea * 0.005;

			for (auto i = 0; i < profiles->count(); i++) {
				auto profile = profiles->item(i);
				if (!profile)
					return false;

				auto areaProps = profile->areaProperties();
				if (!areaProps)
					return false;

				auto area = areaProps->area();
				if (area > (boltArea - boltDelta) && area < (boltArea + boltDelta))
					continue;

				auto extrudeInput = extrudes->createInput(profile, FeatureOperations::NewBodyFeatureOperation);
				if (!extrudeInput)
					return false;

				extrudeInput->setSymmetricExtent(ValueInput::createByReal(values->ballRadius() / 2), false, 0);

				auto extrude = extrudes->add(extrudeInput);
			}
		}
	}

	bool CommandExecuted::createJointBall(Ptr<Component> component, shared_ptr<Values> values) {
		auto planes = component->constructionPlanes();
		if (!planes)
			return false;

		auto planeInput = planes->createInput(component->xZConstructionPlane());
		if (!planeInput)
			return false;

		planeInput->setByOffset(component->xZConstructionPlane(), ValueInput::createByReal(values->ballZ()));

		auto plane = planes->add(planeInput);
		if (!plane)
			return false;

		plane->name("Joint Balls");

		auto sketches = component->sketches();
		if (!sketches)
			return false;

		for (auto row = 1; row <= values->rows(); row++) {
			for (auto col = 1; col <= values->cols(); col++) {
				if (values->jointType(row, col) != ARMATURE_JOINT_OPTION_BALL)
					continue;
				
				auto sketch = sketches->add(plane);
				if (!sketch)
					return false;

				if (!sketch->name("Ball Circles"))
					return false;

				auto curves = sketch->sketchCurves();
				if (!curves)
					return false;

				auto circles = curves->sketchCircles();
				if (!circles)
					return false;

				auto ballDiameterCircle = circles->addByCenterRadius(
					Point3D::create(
						values->ballX(col),
						values->ballY(row),
						0
					),
					values->ballRadius()
				);
				if (!ballDiameterCircle)
					return false;

				auto ballLines = curves->sketchLines();
				if (!ballLines)
					return false;

				auto ballLine = ballLines->addByTwoPoints(
					Point3D::create(values->ballX(col), values->ballY(row) - values->ballRadius(), 0),
					Point3D::create(values->ballX(col), values->ballY(row) + values->ballRadius(), 0)
				);

				if (!ballLine)
					return false;

				auto ballProfiles = sketch->profiles();
				if (!ballProfiles)
					return false;

				auto ballProfile = ballProfiles->item(0);
				if (!ballProfile)
					return false;

				auto features = component->features();
				if (!features)
					return false;

				auto revolves = features->revolveFeatures();
				if (!revolves)
					return false;

				auto revolveInput = revolves->createInput(ballProfile, ballLine, FeatureOperations::NewBodyFeatureOperation);
				if (!revolveInput)
					return false;

				if (!revolveInput->setAngleExtent(false, ValueInput::createByString("360.0 deg")))
					return false;

				auto revolve = revolves->add(revolveInput);
				if (!revolve)
					return false;

				auto bodies = revolve->bodies();
				if (!bodies)
					return false;

				for (auto i = 0; i < bodies->count(); i++) {
					auto body = bodies->item(i);
					body->name("Ball_" + std::to_string(row) + "_" + std::to_string(col) + "_" + std::to_string(i));
				}

				auto ballHolePlaneInput = planes->createInput();
				if (!ballHolePlaneInput)
					return false;

				if (!ballHolePlaneInput->setByAngle(ballLine, ValueInput::createByString("90.0 deg"), ballProfile))
					return false;

				auto ballHolePlane = planes->add(ballHolePlaneInput);
				if (!ballHolePlane)
					return false;

				if (!ballHolePlane->name("Ball Screw Hole"))
					return false;

				auto holeSketch = sketches->add(ballHolePlane);
				if (!holeSketch)
					return false;

				if (!holeSketch->name("Ball Screw Hole"))
					return false;

				auto holeCurves = holeSketch->sketchCurves();
				if (!holeCurves)
					return false;

				auto holeCircles = holeCurves->sketchCircles();
				if (!holeCircles)
					return false;

				auto ballHoleCircle = holeCircles->addByCenterRadius(
					Point3D::create(0, 0, 0),
					values->holeRadius(row, col)
				);
				if (!ballHoleCircle)
					return false;

				auto holeProfiles = holeSketch->profiles();
				if (!holeProfiles)
					return false;

				auto holeProfile = holeProfiles->item(0);
				if (!holeProfile)
					return false;

				auto extrudes = features->extrudeFeatures();
				if (!extrudes)
					return false;

				auto holeExtrudeInput = extrudes->createInput(holeProfile, FeatureOperations::CutFeatureOperation);
				if (!holeExtrudeInput)
					return false;

				auto radius = values->ballRadius();
				if (col % 2 != 0)
					radius = -radius;

				holeExtrudeInput->setDistanceExtent(false, ValueInput::createByReal(radius));

				auto holeExtrude = extrudes->add(holeExtrudeInput);
				if (!holeExtrude)
					return false;
			}
		}
	}

	void CommandExecuted::notify(const Ptr<CommandEventArgs>& eventArgs) {
		if (!eventArgs)
			return;

		auto command = eventArgs->command();
		if (!command)
			return;

		auto inputs = command->commandInputs();

		auto values = Values::create(command->commandInputs());
		if (!values)
			return;

		auto prod = app->activeProduct();
		if (!prod)
			return;

		auto design = static_cast<Ptr<Design>>(prod);
		if (!design)
			return;

		auto comp = design->rootComponent();
		if (!comp)
			return;

		auto occurrences = comp->occurrences();
		if (!occurrences)
			return;

		auto occur = occurrences->addNewComponent(Matrix3D::create());
		if (!occur)
			return;

		auto component = occur->component();
		if (!component)
			return;

		if (!component->name(values->name()))
			return;

		auto planes = component->constructionPlanes();
		if (!planes)
			return;

		auto planeInput = planes->createInput(component->xZConstructionPlane());
		if (!planeInput)
			return;

		planeInput->setByOffset(component->xZConstructionPlane(), ValueInput::createByReal(values->ballZ() + values->plateOffset()));

		auto plane = planes->add(planeInput);
		if (!plane)
			return;

		if (!plane->name("Joint Top Offset"))
			return;

		auto bottom = JointPlate::create(component, component->xZConstructionPlane(), values, false);
		if (!bottom)
			return;

		auto top = JointPlate::create(component, plane, values, true);
		if (!top)
			return;

		if (!createJointBall(component, values))
			return;

		if (!createJointNuts(component, values))
			return;
	}
}