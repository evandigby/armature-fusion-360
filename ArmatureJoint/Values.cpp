#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string>
#include <algorithm>

#include "Values.h"
#include "UI.h"

namespace ArmatureJoint {
	Ptr<UnitsManager> Values::unitsManager;

	double Values::circleRadiusOfSphere(double sphereRadius, double offset) {
		return sqrt(pow(sphereRadius, 2) - pow(offset, 2));
	}

	double Values::diameterForCircleRadiusOfSphere(double circleRadius, double offset) {
		return sqrt(pow(circleRadius, 2) + pow(offset, 2)) * 2;
	}

	double Values::defaultLength() {
		return unitsManager->convert(15, "mm", unitsManager->internalUnits());
	}

	double Values::defaultWidth() {
		return unitsManager->convert(6, "mm", unitsManager->internalUnits());
	}

	double Values::defaultThickness() {
		return unitsManager->convert((double)1 / (double)16, "in", unitsManager->internalUnits());
	}

	double Values::defaultBallDiameter() {
		return unitsManager->convert(5, "mm", unitsManager->internalUnits());
	}

	double Values::defaultRows() {
		return 2;
	}

	double Values::defaultCols() {
		return 1;
	}

	double Values::defaultHoleDiameter() {
		return unitsManager->convert(3, "mm", unitsManager->internalUnits());
	}

	double Values::defaultBoltHoleDiameter() {
		return unitsManager->convert(3, "mm", unitsManager->internalUnits());
	}

	shared_ptr<Values> Values::create(Ptr<CommandInputs> inputs) {
		shared_ptr<Values> values(new Values());

		if (!inputs)
			return nullptr;

		values->nameInput = inputs->itemById(ARMATURE_JOINT_COMMAND_NAME_INPUT_ID);
		if (!values->nameInput)
			return nullptr;

		values->lengthInput = inputs->itemById(ARMATURE_JOINT_COMMAND_LENGTH_INPUT_ID);
		if (!values->lengthInput)
			return nullptr;

		values->widthInput = inputs->itemById(ARMATURE_JOINT_COMMAND_WIDTH_INPUT_ID);
		if (!values->widthInput)
			return nullptr;

		values->thicknessInput = inputs->itemById(ARMATURE_JOINT_COMMAND_PLATE_THICKNESS_INPUT_ID);
		if (!values->thicknessInput)
			return nullptr;

		values->ballDiameterInput = inputs->itemById(ARMATURE_JOINT_COMMAND_BALL_DIAMETER_INPUT_ID);
		if (!values->ballDiameterInput)
			return nullptr;

		values->rowsInput = inputs->itemById(ARMATURE_JOINT_COMMAND_ROWS_INPUT_ID);
		if (!values->rowsInput)
			return nullptr;

		values->colsInput = inputs->itemById(ARMATURE_JOINT_COMMAND_COLS_INPUT_ID);
		if (!values->colsInput)
			return nullptr;

		values->boltHoleInput = inputs->itemById(ARMATURE_JOINT_COMMAND_BOLT_HOLE_DIAMETER_INPUT_ID);
		if (!values->boltHoleInput)
			return nullptr;

		values->tableInput = inputs->itemById(ARMATURE_JOINT_COMMAND_TABLE_INPUT_ID);
		if (!values->tableInput)
			return nullptr;

		values->tableInput->numberOfColumns(values->cols() * 2);

		for (auto i = values->tableInput->rowCount(); i >= values->rows() * 2; i--)
		{
			values->tableInput->deleteRow(i);
		}

		auto tableInputs = values->tableInput->commandInputs();

		for (auto col = 0; col < values->cols(); col++) {
			for (auto row = 0; row < values->rows(); row++) {

				auto typeID = "tableInputType_" + std::to_string(col) + "_" + std::to_string(row);

				auto tableInputType = static_cast<Ptr<RadioButtonGroupCommandInput>>(tableInputs->itemById(typeID));
				if (!tableInputType) {
					auto radio = tableInputs->addRadioButtonGroupCommandInput(typeID, "Type");
					if (!radio)
						return nullptr;

					auto items = radio->listItems();
					if (!items)
						return nullptr;

					auto ball = items->add(ARMATURE_JOINT_OPTION_BALL, true);
					auto nut = items->add(ARMATURE_JOINT_OPTION_NUT, false);
					auto none = items->add(ARMATURE_JOINT_OPTION_NONE, false);

					values->tableInput->addCommandInput(radio, row * 2, col);
				}


				auto holeDiameterID = "tableInputHoleDiameter_" + std::to_string(col) + "_" + std::to_string(row);
				auto tableInputHoleDiameter = static_cast<Ptr<DistanceValueCommandInput>>(tableInputs->itemById(holeDiameterID));
				if (!tableInputHoleDiameter) {
					auto holeDiameter = tableInputs->addDistanceValueCommandInput(
						holeDiameterID,
						"Hole Diameter",
						ValueInput::createByReal(values->defaultHoleDiameter())
					);
					if (!holeDiameter)
						return false;

					auto cmdInput = values->tableInput->addCommandInput(holeDiameter, (row * 2) + 1, col);
					if (!cmdInput)
						return false;
				}
			}
		}

		return values;
	}

	std::string Values::jointType(int row, int col) {
		row--;
		col--;

		auto input = static_cast<Ptr<RadioButtonGroupCommandInput>>(tableInput->getInputAtPosition((row * 2), col));
		if (!input)
			return ARMATURE_JOINT_OPTION_NONE;

		auto selection = input->selectedItem();
		if (!selection)
			return ARMATURE_JOINT_OPTION_NONE;

		return selection->name();
	}

	double Values::ballDiameter() {
		if (!ballDiameterInput)
			return 0;

		return ballDiameterInput->value();
	}

	double Values::width() {
		if (!widthInput)
			return 0;

		return widthInput->value();
	}

	double Values::length() {
		if (!lengthInput)
			return 0;
		return lengthInput->value();
	}

	double Values::thickness() {
		if (!thicknessInput)
			return 0;

		return thicknessInput->value();
	}

	double Values::chamferLength() {
		return ballRadius() / 6;
	}
	double Values::chamferAngle() {
		return M_PI_4; // 45 degrees in radians
	}

	void Values::setExtents() {
		lengthInput->setManipulator(Point3D::create(0, 0, 0), Vector3D::create(1, 0, 0));
		widthInput->setManipulator(Point3D::create(0, 0, 0), Vector3D::create(0, 0, 1));
		thicknessInput->setManipulator(Point3D::create(0, 0, 0), Vector3D::create(0, 1, 0));
		ballDiameterInput->setManipulator(Point3D::create(ballX(1), ballZ(), -ballY(1)), Vector3D::create(0, 1, 0));

		if (width() < minWidth()) {
			widthInput->value(minWidth());
		}

		lengthInput->minimumValue(minWidth());
		widthInput->minimumValue(minWidth());
		ballDiameterInput->maximumValue(maxBallDiameter());
	}

	double Values::ballOffset() {
		return ballRadius() / 1.2;
	}

	double Values::ballRadius() {
		return ballDiameter() / 2;
	}

	double Values::ballX(int col) {
		if (col == 1)
			return ballRadius() / 1.25;

		return length() - (ballRadius() / 1.25);
	}

	double Values::ballY(int row) {
		auto rowSize = width() / (double)rows();

		return -((rowSize * row) - (rowSize / 2));
	}

	double Values::plateOffset() {
		return ballOffset() - (chamferLength() / 1.25);
	}

	double Values::ballZ() {
		return thickness() + plateOffset();
	}

	double Values::circleRadius() {
		return circleRadiusOfSphere(ballRadius(), ballOffset());
	}

	double Values::circleArea() {
		return M_PI * pow(circleRadius(), 2);
	}

	double Values::circleCircumference() {
		return 2 * M_PI * circleRadius();
	}

	double Values::minWidth() {
		return ((circleRadius() * 2) + 0.05) * rows();
	}

	double Values::maxBallDiameter() {
		return diameterForCircleRadiusOfSphere((width() / rows()) + 0.05, ballOffset());
	}

	double Values::expectedArea() {
		return (length() * width()) - (boltCircleArea() + (circleArea() * (double)numJointTypes(ARMATURE_JOINT_OPTION_BALL)));
	}

	int Values::numJointTypes(std::string expectedJointType) {
		int num = 0;
		for (auto row = 1; row <= rows(); row++) {
			for (auto col = 1; col <= cols(); col++) {
				auto t = jointType(row, col);
				if (t == expectedJointType)
					num++;
			}
		}
		return num;
	}

	int Values::rows() {
		if (!rowsInput)
			return 1;

		return rowsInput->value();
	}

	int Values::cols() {
		if (!colsInput)
			return 1;
		return colsInput->value();
	}

	std::string Values::name() {
		if (!nameInput)
			return "";

		return nameInput->value();
	}
	double Values::holeDiameter(int row, int col) {
		row--;
		col--;

		auto input = static_cast<Ptr<DistanceValueCommandInput>>(tableInput->getInputAtPosition((row * 2) + 1, col));
		if (!input)
			return 0;
		return input->value();
	}

	double Values::holeRadius(int row, int col) {
		return holeDiameter(row, col) / 2;
	}

	double Values::boltHoleDiameter() {
		if (!boltHoleInput)
			return 0;

		return boltHoleInput->value();
	}

	double Values::boltHoleRadius() {
		return boltHoleDiameter() / 2;
	}

	double Values::boltCircleArea() {
		return  M_PI * pow(boltHoleRadius(), 2);
	}
}
