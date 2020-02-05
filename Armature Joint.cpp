
#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>
#define _USE_MATH_DEFINES
#include <math.h>


using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;

Ptr<Application> app;
Ptr<UserInterface> ui;
Ptr<CommandDefinition> button;
Ptr<CommandControl> control;

#define ARMATURE_COMMAND_ID "createArmatureJoint"
#define ARMATURE_COMMAND_LENGTH_INPUT_ID "armatureJointLengthInputID"
#define ARMATURE_COMMAND_WIDTH_INPUT_ID "armatureJointWidthInputID"
#define ARMATURE_COMMAND_PLATE_THICKNESS_INPUT_ID "armatureJointPlateThicknesshInputID"
#define ARMATURE_COMMAND_BALL_DIAMETER_INPUT_ID "armatureJointBallDiameterInputID"

double circleRadiusOfSphere(double sphereDiameter, double offset) {
	double sphereRadius = sphereDiameter / 2;

	return sqrt(pow(sphereRadius, 2) - pow(offset, 2));
}

double diameterForCircleRadiusOfSphere(double circleRadius, double offset) {
	return sqrt(pow(circleRadius, 2) + pow(offset, 2));
}

typedef struct ArmatureValues {
	double length;
	double width;
	double thickness;

	double ballDiameter;
	double ballRadius;
	double ballX;
	double ballY;
	double ballZ;
	double ballOffset;

	double circleRadius;
	double circleArea;

	double minWidth;
	double maxBallDiameter;
};

bool loadArmatureValues(Ptr<CommandInputs> inputs, ArmatureValues* values) {
	if (!inputs)
		return false;

	auto lengthInput = static_cast<Ptr<DistanceValueCommandInput>>(inputs->itemById(ARMATURE_COMMAND_LENGTH_INPUT_ID));
	if (!lengthInput)
		return false;

	auto widthInput = static_cast<Ptr<DistanceValueCommandInput>>(inputs->itemById(ARMATURE_COMMAND_WIDTH_INPUT_ID));
	if (!widthInput)
		return false;

	auto thicknessInput = static_cast<Ptr<DistanceValueCommandInput>>(inputs->itemById(ARMATURE_COMMAND_PLATE_THICKNESS_INPUT_ID));
	if (!thicknessInput)
		return false;

	auto ballDiameterInput = static_cast<Ptr<DistanceValueCommandInput>>(inputs->itemById(ARMATURE_COMMAND_BALL_DIAMETER_INPUT_ID));
	if (!ballDiameterInput)
		return false;

	values->width = widthInput->value();
	values->length = lengthInput->value();
	values->thickness = thicknessInput->value();

	values->ballDiameter = ballDiameterInput->value();
	values->ballOffset = values->ballDiameter / 3;
	values->ballRadius = values->ballDiameter / 2;
	values->ballX = values->length - values->ballRadius;
	values->ballY = -(values->width / 2);
	values->ballZ = values->thickness + values->ballRadius - (values->ballOffset / 2);

	values->circleRadius = circleRadiusOfSphere(values->ballDiameter, values->ballOffset);
	values->circleArea = M_PI * pow(values->circleRadius, 2);

	values->minWidth = (values->circleRadius * 2) + 0.05;
	values->maxBallDiameter = diameterForCircleRadiusOfSphere(values->width + 0.05, values->ballOffset);

	return true;
}

class ArmatureCommandExecuted : public CommandEventHandler {
public:

	bool createJointPlate(Ptr<Component> component, Ptr<ConstructionPlane> plane, ArmatureValues *values) {
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

		auto rectangle = lines->addTwoPointRectangle(Point3D::create(0, 0, 0), Point3D::create(values->length, -values->width, 0));
		if (!rectangle)
			return false;
		auto rectangleArea = values->length * values->width;

		auto circles = curves->sketchCircles();
		if (!circles)
			return false;

		auto ballCircle = circles->addByCenterRadius(
			Point3D::create(
				values->ballX,
				values->ballY,
				0
			),
			values->circleRadius
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
		auto expectedArea = rectangleArea - values->circleArea;
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

		auto thicknessInput = ValueInput::createByReal(values->thickness);
		if (!thicknessInput)
			return false;

		auto extrude = extrudes->addSimple(profile, thicknessInput, FeatureOperations::NewBodyFeatureOperation);
		if (!extrude)
			return false;

		return true;
	}

	bool createJointBall(Ptr<Component> component, ArmatureValues *values) {
		auto planes = component->constructionPlanes();
		if (!planes)
			return false;

		auto planeInput = planes->createInput(component->xZConstructionPlane());
		if (!planeInput)
			return false;

		planeInput->setByOffset(component->xZConstructionPlane(), ValueInput::createByReal(values->ballZ));

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
				values->ballX,
				values->ballY,
				0
			),
			values->ballRadius
		);

		auto ballLines = curves->sketchLines();
		if (!ballLines)
			return false;

		auto ballLine = ballLines->addByTwoPoints(
			Point3D::create(values->ballX, values->ballY - values->ballRadius, 0),
			Point3D::create(values->ballX, values->ballY + values->ballRadius, 0)
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

	void notify(const Ptr<CommandEventArgs>& eventArgs) override {
		if (!eventArgs)
			return;

		auto command = eventArgs->command();
		if (!command)
			return;

		auto inputs = command->commandInputs();

		ArmatureValues values;
		if (!loadArmatureValues(command->commandInputs(), &values))
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

		planeInput->setByOffset(component->xZConstructionPlane(), ValueInput::createByReal(values.thickness + (values.ballDiameter - values.ballOffset)));

		auto plane = planes->add(planeInput);
		if (!plane)
			return;

		if (!createJointPlate(component, component->xZConstructionPlane(), &values))
			return;

		if (!createJointPlate(component, plane, &values))
			return;

		if (!createJointBall(component, &values))
			return;
	}
};

class ArmatureCommandInputChanged : public InputChangedEventHandler {
	ArmatureValues* values;

	void notify(const Ptr<InputChangedEventArgs>& eventArgs) override {
		if (!eventArgs)
			return;

		auto inputs = eventArgs->inputs();
		if (!inputs)
			return;

		if (!loadArmatureValues(inputs, values))
			return;

		auto ballDiameterInput = static_cast<Ptr<DistanceValueCommandInput>>(inputs->itemById(ARMATURE_COMMAND_BALL_DIAMETER_INPUT_ID));
		if (!ballDiameterInput)
			return;

		auto widthInput = static_cast<Ptr<DistanceValueCommandInput>>(inputs->itemById(ARMATURE_COMMAND_WIDTH_INPUT_ID));
		if (!widthInput)
			return;

		ballDiameterInput->setManipulator(Point3D::create(values->ballX, values->ballZ, -values->ballY), Vector3D::create(0, 1, 0));
		widthInput->minimumValue(values->minWidth);
		ballDiameterInput->maximumValue(values->maxBallDiameter);

	}

public:
	void setValues(ArmatureValues* _values) {
		values = _values;
	}

};

class ArmatureCommandCreated : public CommandCreatedEventHandler {
private:
	ArmatureValues values;
public:
	ArmatureCommandCreated(double length, double width, double thickness, double ballDiameter) {
		values.length = length;
		values.width = width;
		values.thickness = thickness;
		values.ballDiameter = ballDiameter;
	}

	void notify(const Ptr<CommandCreatedEventArgs>& eventArgs) override {
		if (!eventArgs)
			return;

		auto cmd = eventArgs->command();
		if (!cmd)
			return;

		auto inputs = cmd->commandInputs();
		
		auto lengthInput = inputs->addDistanceValueCommandInput(
			ARMATURE_COMMAND_LENGTH_INPUT_ID,
			"Joint Length",
			ValueInput::createByReal(values.length)
		);

		lengthInput->setManipulator(Point3D::create(0, 0, 0), Vector3D::create(1, 0, 0));

		auto widthInput = inputs->addDistanceValueCommandInput(
			ARMATURE_COMMAND_WIDTH_INPUT_ID,
			"Joint Width",
			ValueInput::createByReal(values.width)
		);

		auto plateThicknessInput = inputs->addDistanceValueCommandInput(
			ARMATURE_COMMAND_PLATE_THICKNESS_INPUT_ID,
			"Plate Thickness",
			ValueInput::createByReal(values.thickness)
		);

		auto ballDiameterInput = inputs->addDistanceValueCommandInput(
			ARMATURE_COMMAND_BALL_DIAMETER_INPUT_ID,
			"Ball Diameter",
			ValueInput::createByReal(values.ballDiameter)
		);

		if (!loadArmatureValues(inputs, &values))
			return;

		widthInput->minimumValue(values.minWidth);
		widthInput->setManipulator(Point3D::create(0, 0, 0), Vector3D::create(0, 0, 1));
		plateThicknessInput->setManipulator(Point3D::create(0, 0, 0), Vector3D::create(0, 1, 0));
		ballDiameterInput->setManipulator(Point3D::create(values.ballX, values.ballZ, -values.ballY), Vector3D::create(0, 1, 0));
		ballDiameterInput->maximumValue(values.maxBallDiameter);

		_onInputChanged.setValues(&values);

		auto inputChangedEvent = cmd->inputChanged();
		if (!inputChangedEvent->add(&_onInputChanged))
			return;

		auto onPreview = cmd->executePreview();
		if (!onPreview)
			return;
		onPreview->add(&_onExecute);

		auto onExec = cmd->execute();
		if (!onExec)
			return;

		onExec->add(&_onExecute);
	}

private:
	ArmatureCommandExecuted _onExecute;
	ArmatureCommandInputChanged _onInputChanged;

};

ArmatureCommandCreated* commandCreatedEvent;


extern "C" XI_EXPORT bool run(const char* context)
{
	app = Application::get();
	if (!app)
		return false;

	ui = app->userInterface();
	if (!ui)
		return false;

	auto definitions = ui->commandDefinitions();
	if (!definitions)
		return false;

	button = definitions->addButtonDefinition(
		ARMATURE_COMMAND_ID,
		"Create Armature",
		"Creates a stop motion animation armature",
		""
	);

	if (!button) {
		return false;
	}

	auto prod = app->activeProduct();
	if (!prod)
		return false;

	auto design = static_cast<Ptr<Design>>(prod);
	if (!design)
		return false;

	auto unitsManager = design->unitsManager();
	if (!unitsManager)
		return false;

	auto length = unitsManager->convert(15, "mm", unitsManager->internalUnits());
	auto width = unitsManager->convert(5, "mm", unitsManager->internalUnits());
	auto plateThickness = unitsManager->convert((double)1 / (double)16, "in", unitsManager->internalUnits());
	auto ballDiameter = width / 1.5;

	auto commandCreated = button->commandCreated();
	if (!commandCreated)
		return false;

	commandCreatedEvent = new ArmatureCommandCreated(length, width, plateThickness, ballDiameter);
	commandCreated->add(commandCreatedEvent);

	auto panels = ui->allToolbarPanels();
	if (!panels)
		return false;

	auto panel = panels->itemById("SolidScriptsAddinsPanel");
	if (!panel) {
		return false;
	}

	auto controls = panel->controls();
	if (!controls)
		return false;

	control = controls->addCommand(button);

	return true;
}

extern "C" XI_EXPORT bool stop(const char* context)
{
	if (commandCreatedEvent) {
		delete commandCreatedEvent;
		commandCreatedEvent = nullptr;
	}
	if (control) {
		control->deleteMe();
		control = nullptr;
	}

	if (button) {
		button->deleteMe();
		button = nullptr;
	}

	if (ui)
		ui = nullptr;

	return true;
}


#ifdef XI_WIN

#include <windows.h>

BOOL APIENTRY DllMain(HMODULE hmodule, DWORD reason, LPVOID reserved)
{
	switch (reason)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

#endif // XI_WIN
