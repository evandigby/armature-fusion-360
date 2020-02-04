
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

class ArmatureCommandExecuted : public CommandEventHandler {
public:
	void notify(const Ptr<CommandEventArgs>& eventArgs) override {
		if (!eventArgs)
			return;

		auto command = eventArgs->command();
		if (!command)
			return;

		auto in = command->commandInputs();
		if (!in)
			return;

		auto length = static_cast<Ptr<DistanceValueCommandInput>>(in->itemById(ARMATURE_COMMAND_LENGTH_INPUT_ID));
		if (!length)
			return;

		auto width = static_cast<Ptr<DistanceValueCommandInput>>(in->itemById(ARMATURE_COMMAND_WIDTH_INPUT_ID));
		if (!width)
			return;

		auto plateThickness = static_cast<Ptr<DistanceValueCommandInput>>(in->itemById(ARMATURE_COMMAND_PLATE_THICKNESS_INPUT_ID));
		if (!plateThickness)
			return;

		auto ballDiameter = static_cast<Ptr<DistanceValueCommandInput>>(in->itemById(ARMATURE_COMMAND_BALL_DIAMETER_INPUT_ID));
		if (!ballDiameter)
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

		if (!component->name("joint"))
			return;

		auto sketches = component->sketches();
		if (!sketches)
			return;

		auto sketch = sketches->add(component->xZConstructionPlane());
		if (!sketch)
			return;

		auto curves = sketch->sketchCurves();
		if (!curves)
			return;

		auto lines = curves->sketchLines();
		if (!lines)
			return;

		auto rectangle = lines->addTwoPointRectangle(Point3D::create(0, 0, 0), Point3D::create(length->value(), -width->value(), 0));
		if (!rectangle)
			return;
		auto rectangleArea = length->value() * width->value();

		auto circles = curves->sketchCircles();
		if (!circles)
			return;

		auto ballOffset =  ballDiameter->value() / 3;
		auto circleRadius = circleRadiusOfSphere(ballDiameter->value(), ballOffset);
		auto circleArea = M_PI*pow(circleRadius, 2);

		auto ballCircle = circles->addByCenterRadius(
			Point3D::create(
				length->value() - (ballDiameter->value() / 2), 
				-(width->value() / 2), 
				0
			),
			circleRadius
		);

		if (!ballCircle)
			return;

		auto features = component->features();
		if (!features)
			return;

		auto extrudes = features->extrudeFeatures();
		if (!extrudes)
			return;

		auto profiles = sketch->profiles();
		if (!profiles || profiles->count() < 1)
			return;

		auto accuracy = CalculationAccuracy::LowCalculationAccuracy;
		auto expectedArea = rectangleArea - circleArea;
		auto delta = expectedArea * 0.005; // Low calculation accuracy is within 0.5%

		Ptr<Profile> profile;
		for (auto i = 0; i < profiles->count(); i++)
		{
			auto current = profiles->item(i);
			if (!current)
				return;

			auto areaProps = current->areaProperties(accuracy);
			if (!areaProps)
				return;


			auto area = areaProps->area();
			if (area > expectedArea - delta && area < expectedArea + delta) {
				profile = current;
				break;
			}
		}
		if (!profile)
			return;

		auto thicknessInput = ValueInput::createByReal(plateThickness->value());
		if (!thicknessInput)
			return;

		auto extrude = extrudes->addSimple(profile, thicknessInput, FeatureOperations::NewBodyFeatureOperation);
		if (!extrude)
			return;
	}
};

class ArmatureCommandCreated : public CommandCreatedEventHandler {
public:
	void notify(const Ptr<CommandCreatedEventArgs>& eventArgs) override {
		if (!eventArgs)
			return;

		auto cmd = eventArgs->command();
		if (!cmd)
			return;

		auto inputs = cmd->commandInputs();

		auto prod = app->activeProduct();
		if (!prod)
			return;

		auto design = static_cast<Ptr<Design>>(prod);
		if (!design)
			return;

		auto unitsManager = design->unitsManager();
		if (!unitsManager)
			return;

		auto length = unitsManager->convert(15, "mm", unitsManager->internalUnits());
		auto lengthInput = inputs->addDistanceValueCommandInput(
			ARMATURE_COMMAND_LENGTH_INPUT_ID,
			"Joint Length",
			ValueInput::createByReal(length)
		);

		lengthInput->setManipulator(Point3D::create(0, 0, 0), Vector3D::create(1, 0, 0));

		auto width = unitsManager->convert(5, "mm", unitsManager->internalUnits());
		auto widthInput = inputs->addDistanceValueCommandInput(
			ARMATURE_COMMAND_WIDTH_INPUT_ID,
			"Joint Width",
			ValueInput::createByReal(width)
		);

		widthInput->setManipulator(Point3D::create(0, 0, 0), Vector3D::create(0, 0, -1));

		auto plateThickness = unitsManager->convert((double)1/(double)16, "in", unitsManager->internalUnits());
		auto plateThicknessInput = inputs->addDistanceValueCommandInput(
			ARMATURE_COMMAND_PLATE_THICKNESS_INPUT_ID,
			"Plate Thickness",
			ValueInput::createByReal(plateThickness)
		);

		plateThicknessInput->setManipulator(Point3D::create(0, 0, 0), Vector3D::create(0, 1, 0));

		auto ballDiameterInput = inputs->addDistanceValueCommandInput(
			ARMATURE_COMMAND_BALL_DIAMETER_INPUT_ID,
			"Ball Diameter",
			ValueInput::createByReal(width)
		);

		ballDiameterInput->setManipulator(Point3D::create(0, 0, width/2), Vector3D::create(0, 1, 0));

		auto onExec = cmd->execute();
		if (!onExec)
			return;

		onExec->add(&_onExecute);
	}

private:
	ArmatureCommandExecuted _onExecute;

} _armatureCommandCreated;

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

	auto commandCreated = button->commandCreated();
	if (!commandCreated)
		return false;

	commandCreated->add(&_armatureCommandCreated);

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
