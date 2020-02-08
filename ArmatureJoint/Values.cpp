#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "Values.h"
#include "UI.h"

namespace ArmatureJoint {
	double Values::circleRadiusOfSphere(double sphereDiameter, double offset) {
		double sphereRadius = sphereDiameter / 2;

		return sqrt(pow(sphereRadius, 2) - pow(offset, 2));
	}

	double Values::diameterForCircleRadiusOfSphere(double circleRadius, double offset) {
		return sqrt(pow(circleRadius, 2) + pow(offset, 2));
	}

	double Values::defaultLength(Ptr<UnitsManager> unitsManager) {
		return unitsManager->convert(15, "mm", unitsManager->internalUnits());
	}

	double Values::defaultWidth(Ptr<UnitsManager> unitsManager) {
		return unitsManager->convert(5, "mm", unitsManager->internalUnits());
	}

	double Values::defaultThickness(Ptr<UnitsManager> unitsManager) {
		return unitsManager->convert((double)1 / (double)16, "in", unitsManager->internalUnits());
	}

	double Values::defaultBallDiameter(Ptr<UnitsManager> unitsManager) {
		return defaultWidth(unitsManager) / 1.5;
	}

	double Values::defaultRows(Ptr<UnitsManager> unitsManager) {
		return 2;
	}

	double Values::defaultCols(Ptr<UnitsManager> unitsManager) {
		return 1;
	}

	shared_ptr<Values> Values::create(Ptr<CommandInputs> inputs) {
		shared_ptr<Values> values(new Values());

		if (!inputs)
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

		auto test = inputs->itemById(ARMATURE_JOINT_COMMAND_ROWS_INPUT_ID);
		values->rowsInput = inputs->itemById(ARMATURE_JOINT_COMMAND_ROWS_INPUT_ID);
		if (!values->rowsInput)
			return nullptr;

		values->colsInput = inputs->itemById(ARMATURE_JOINT_COMMAND_COLS_INPUT_ID);
		if (!values->colsInput)
			return nullptr;

		return values;
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
		return ballDiameter() / 3;
	}

	double Values::ballRadius() {
		return ballDiameter() / 2;
	}

	double Values::ballX(int col) {
		if (col == 1)
			return ballRadius();

		return length() - ballRadius();
	}

	double Values::ballY(int row) {
		auto rowSize = width() / (double)rows();

		return -((rowSize * row) - (rowSize / 2));
	}

	double Values::ballZ() {
		return thickness() + ballRadius() - (ballOffset() / 2);
	}

	double Values::circleRadius() {
		return circleRadiusOfSphere(ballDiameter(), ballOffset());
	}

	double Values::circleArea() {
		return M_PI * pow(circleRadius(), 2);
	}

	double Values::minWidth() {
		return ((circleRadius() * 2) + 0.05) * rows();
	}

	double Values::maxBallDiameter() {
		return diameterForCircleRadiusOfSphere((width() / rows()) + 0.05, ballOffset());
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
}