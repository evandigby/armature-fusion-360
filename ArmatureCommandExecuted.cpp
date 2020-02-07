#include "ArmatureCommandExecuted.h"

#define _USE_MATH_DEFINES
#include <math.h>

bool ArmatureCommandExecuted::createJointPlate(Ptr<Component> component, Ptr<ConstructionPlane> plane, shared_ptr<ArmatureValues> values) {
	if (!component->name("joint"))
		return false;

	auto sketches = component->sketches();
	if (!sketches)
		return false;

	auto sketch = sketches->add(plane);
	if (!sketch)
		return false;

	auto curves = sketch->sketchCurves();
	if (!curves)
		return false;

	auto lines = curves->sketchLines();
	if (!lines)
		return false;

	auto rectangle = lines->addTwoPointRectangle(Point3D::create(0, 0, 0), Point3D::create(values->length(), -values->width(), 0));
	if (!rectangle)
		return false;
	auto rectangleArea = values->length() * values->width();

	auto circles = curves->sketchCircles();
	if (!circles)
		return false;

	auto ballCircle = circles->addByCenterRadius(
		Point3D::create(
			values->ballX(),
			values->ballY(),
			0
		),
		values->circleRadius()
	);

	if (!ballCircle)
		return false;

	auto features = component->features();
	if (!features)
		return false;

	auto extrudes = features->extrudeFeatures();
	if (!extrudes)
		return false;

	auto profiles = sketch->profiles();
	if (!profiles || profiles->count() < 1)
		return false;

	auto accuracy = CalculationAccuracy::LowCalculationAccuracy;
	auto expectedArea = rectangleArea - values->circleArea();
	auto delta = expectedArea * 0.005; // Low calculation accuracy is within 0.5%. We don't have many profiles so that's fine.

	Ptr<Profile> profile;
	for (auto i = 0; i < profiles->count(); i++)
	{
		auto current = profiles->item(i);
		if (!current)
			return false;

		auto areaProps = current->areaProperties(accuracy);
		if (!areaProps)
			return false;


		auto area = areaProps->area();
		if (area > expectedArea - delta && area < expectedArea + delta) {
			profile = current;
			break;
		}
	}
	if (!profile)
		return false;

	auto thicknessInput = ValueInput::createByReal(values->thickness());
	if (!thicknessInput)
		return false;

	auto extrude = extrudes->addSimple(profile, thicknessInput, FeatureOperations::NewBodyFeatureOperation);
	if (!extrude)
		return false;

	return true;
}

bool ArmatureCommandExecuted::createJointBall(Ptr<Component> component, shared_ptr<ArmatureValues> values) {
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

	auto sketches = component->sketches();
	if (!sketches)
		return false;

	auto sketch = sketches->add(plane);
	if (!sketch)
		return false;

	auto curves = sketch->sketchCurves();
	if (!curves)
		return false;

	auto circles = curves->sketchCircles();
	if (!circles)
		return false;

	auto ballDiameterCircle = circles->addByCenterRadius(
		Point3D::create(
			values->ballX(),
			values->ballY(),
			0
		),
		values->ballRadius()
	);

	auto ballLines = curves->sketchLines();
	if (!ballLines)
		return false;

	auto ballLine = ballLines->addByTwoPoints(
		Point3D::create(values->ballX(), values->ballY() - values->ballRadius(), 0),
		Point3D::create(values->ballX(), values->ballY() + values->ballRadius(), 0)
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

	revolveInput->setAngleExtent(false, ValueInput::createByReal(2 * M_PI));

	auto revolve = revolves->add(revolveInput);
	if (!revolve)
		return false;
}

void ArmatureCommandExecuted::notify(const Ptr<CommandEventArgs>& eventArgs) {
	if (!eventArgs)
		return;

	auto command = eventArgs->command();
	if (!command)
		return;

	auto inputs = command->commandInputs();

	auto values = ArmatureValues::create(command->commandInputs());
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

	auto planes = component->constructionPlanes();
	if (!planes)
		return;

	auto planeInput = planes->createInput(component->xZConstructionPlane());
	if (!planeInput)
		return;

	planeInput->setByOffset(component->xZConstructionPlane(), ValueInput::createByReal(values->thickness() + (values->ballDiameter() - values->ballOffset())));

	auto plane = planes->add(planeInput);
	if (!plane)
		return;

	if (!createJointPlate(component, component->xZConstructionPlane(), values))
		return;

	if (!createJointPlate(component, plane, values))
		return;

	if (!createJointBall(component, values))
		return;
}
