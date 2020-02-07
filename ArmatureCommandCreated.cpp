#include "ArmatureCommandCreated.h"

#include "ArmatureUI.h"

void ArmatureCommandCreated::notify(const Ptr<CommandCreatedEventArgs>& eventArgs) {
	if (!eventArgs)
		return;

	auto cmd = eventArgs->command();
	if (!cmd)
		return;

	auto prod = app->activeProduct();
	if (!prod)
		return;

	auto design = static_cast<Ptr<Design>>(prod);
	if (!design)
		return;

	auto unitsManager = design->unitsManager();
	if (!unitsManager)
		return;

	auto inputs = cmd->commandInputs();
	if (!inputs)
		return;

	auto lengthInput = inputs->addDistanceValueCommandInput(
		ARMATURE_COMMAND_LENGTH_INPUT_ID,
		"Joint Length",
		ValueInput::createByReal(ArmatureValues::defaultLength(unitsManager))
	);
	if (!lengthInput)
		return;

	auto widthInput = inputs->addDistanceValueCommandInput(
		ARMATURE_COMMAND_WIDTH_INPUT_ID,
		"Joint Width",
		ValueInput::createByReal(ArmatureValues::defaultWidth(unitsManager))
	);
	if (!widthInput)
		return;

	auto plateThicknessInput = inputs->addDistanceValueCommandInput(
		ARMATURE_COMMAND_PLATE_THICKNESS_INPUT_ID,
		"Plate Thickness",
		ValueInput::createByReal(ArmatureValues::defaultThickness(unitsManager))
	);
	if (!plateThicknessInput)
		return;

	auto ballDiameterInput = inputs->addDistanceValueCommandInput(
		ARMATURE_COMMAND_BALL_DIAMETER_INPUT_ID,
		"Ball Diameter",
		ValueInput::createByReal(ArmatureValues::defaultBallDiameter(unitsManager))
	);
	if (!ballDiameterInput)
		return;

	auto values = ArmatureValues::create(inputs);
	values->setExtents();

	auto inputChangedEvent = cmd->inputChanged();
	if (!inputChangedEvent->add(_onInputChanged.get()))
		return;

	auto onPreview = cmd->executePreview();
	if (!onPreview)
		return;
	onPreview->add(_onExecute.get());

	auto onExec = cmd->execute();
	if (!onExec)
		return;
	onExec->add(_onExecute.get());
}