#include "CommandCreated.h"

#include "UI.h"

namespace ArmatureJoint {
	void CommandCreated::notify(const Ptr<CommandCreatedEventArgs>& eventArgs) {
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

		Values::unitsManager = unitsManager;

		auto inputs = cmd->commandInputs();
		if (!inputs)
			return;

		auto nameInput = inputs->addStringValueInput(
			ARMATURE_JOINT_COMMAND_NAME_INPUT_ID,
			"Name",
			"Joint"
		);
		if (!nameInput)
			return;

		auto lengthInput = inputs->addDistanceValueCommandInput(
			ARMATURE_JOINT_COMMAND_LENGTH_INPUT_ID,
			"Joint Length",
			ValueInput::createByReal(Values::defaultLength())
		);
		if (!lengthInput)
			return;

		auto widthInput = inputs->addDistanceValueCommandInput(
			ARMATURE_JOINT_COMMAND_WIDTH_INPUT_ID,
			"Joint Width",
			ValueInput::createByReal(Values::defaultWidth())
		);
		if (!widthInput)
			return;

		auto plateThicknessInput = inputs->addDistanceValueCommandInput(
			ARMATURE_JOINT_COMMAND_PLATE_THICKNESS_INPUT_ID,
			"Plate Thickness",
			ValueInput::createByReal(Values::defaultThickness())
		);
		if (!plateThicknessInput)
			return;

		auto boltHoleDiameterInput = inputs->addDistanceValueCommandInput(
			ARMATURE_JOINT_COMMAND_BOLT_HOLE_DIAMETER_INPUT_ID,
			"Bolt Hole Diameter",
			ValueInput::createByReal(Values::defaultBoltHoleDiameter())
		);
		if (!boltHoleDiameterInput)
			return;

		auto ballDiameterInput = inputs->addDistanceValueCommandInput(
			ARMATURE_JOINT_COMMAND_BALL_DIAMETER_INPUT_ID,
			"Ball Diameter",
			ValueInput::createByReal(Values::defaultBallDiameter())
		);
		if (!ballDiameterInput)
			return;

		auto ballRowsInput = inputs->addIntegerSpinnerCommandInput(
			ARMATURE_JOINT_COMMAND_ROWS_INPUT_ID,
			"Rows",
			1,
			10,
			1,
			ValueInput::createByReal(Values::defaultRows())
		);
		if (!ballRowsInput)
			return;
		
		auto ballColsInput = inputs->addIntegerSpinnerCommandInput(
			ARMATURE_JOINT_COMMAND_COLS_INPUT_ID,
			"Cols",
			1,
			2,
			1,
			ValueInput::createByReal(Values::defaultCols())
		);
		if (!ballColsInput)
			return;

		auto tableInput = inputs->addTableCommandInput(
			ARMATURE_JOINT_COMMAND_TABLE_INPUT_ID,
			"Specifications",
			2,
			"1:1"
		);
		if (!tableInput)
			return;

		auto values = Values::create(inputs);
		if (!values)
			return;

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
}