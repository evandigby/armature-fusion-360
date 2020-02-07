#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "ArmatureValues.h"
#include "ArmatureUI.h"

double ArmatureValues::circleRadiusOfSphere(double sphereDiameter, double offset) {
	double sphereRadius = sphereDiameter / 2;

	return sqrt(pow(sphereRadius, 2) - pow(offset, 2));
}

double ArmatureValues::diameterForCircleRadiusOfSphere(double circleRadius, double offset) {
	return sqrt(pow(circleRadius, 2) + pow(offset, 2));
}

double ArmatureValues::defaultLength(Ptr<UnitsManager> unitsManager) {
	return unitsManager->convert(15, "mm", unitsManager->internalUnits());
}

double ArmatureValues::defaultWidth(Ptr<UnitsManager> unitsManager) {
	return unitsManager->convert(5, "mm", unitsManager->internalUnits());
}

double ArmatureValues::defaultThickness(Ptr<UnitsManager> unitsManager) {
	return unitsManager->convert((double)1 / (double)16, "in", unitsManager->internalUnits());
}

double ArmatureValues::defaultBallDiameter(Ptr<UnitsManager> unitsManager) {
	return defaultWidth(unitsManager) / 1.5;
}

shared_ptr<ArmatureValues> ArmatureValues::create(Ptr<CommandInputs> inputs) {
	shared_ptr<ArmatureValues> values(new ArmatureValues());

	if (!inputs)
		return nullptr;

	values->lengthInput = inputs->itemById(ARMATURE_COMMAND_LENGTH_INPUT_ID);
	if (!values->lengthInput)
		return nullptr;

	values->widthInput = inputs->itemById(ARMATURE_COMMAND_WIDTH_INPUT_ID);
	if (!values->widthInput)
		return nullptr;

	values->thicknessInput = inputs->itemById(ARMATURE_COMMAND_PLATE_THICKNESS_INPUT_ID);
	if (!values->thicknessInput)
		return nullptr;

	values->ballDiameterInput = inputs->itemById(ARMATURE_COMMAND_BALL_DIAMETER_INPUT_ID);
	if (!values->ballDiameterInput)
		return nullptr;

	return values;
}

double ArmatureValues::ballDiameter() {
	if (!ballDiameterInput)
		return 0;

	return ballDiameterInput->value();
}

double ArmatureValues::width() {
	if (!widthInput)
		return 0;

	return widthInput->value();
}

double ArmatureValues::length() {
	if (!lengthInput)
		return 0;
	return lengthInput->value();
}

double ArmatureValues::thickness() {
	if (!thicknessInput)
		return 0;

	return thicknessInput->value();
}

void ArmatureValues::setExtents() {
	lengthInput->setManipulator(Point3D::create(0, 0, 0), Vector3D::create(1, 0, 0));
	widthInput->setManipulator(Point3D::create(0, 0, 0), Vector3D::create(0, 0, 1));
	thicknessInput->setManipulator(Point3D::create(0, 0, 0), Vector3D::create(0, 1, 0));
	ballDiameterInput->setManipulator(Point3D::create(ballX(), ballZ(), -ballY()), Vector3D::create(0, 1, 0));

	lengthInput->minimumValue(minWidth());
	widthInput->minimumValue(minWidth());
	ballDiameterInput->maximumValue(maxBallDiameter());
}

double ArmatureValues::ballOffset() {
	return ballDiameter() / 3;
}

double ArmatureValues::ballRadius() {
	return ballDiameter() / 2;
}

double ArmatureValues::ballX() {
	return length() - ballRadius();
}

double ArmatureValues::ballY() {
	return -(width() / 2);
}

double ArmatureValues::ballZ() {
	return thickness() + ballRadius() - (ballOffset() / 2);
}

double ArmatureValues::circleRadius() {
	return circleRadiusOfSphere(ballDiameter(), ballOffset());
}

double ArmatureValues::circleArea() {
	return M_PI * pow(circleRadius(), 2);
}

double ArmatureValues::minWidth() {
	return (circleRadius() * 2) + 0.05;
}

double ArmatureValues::maxBallDiameter() {
	return diameterForCircleRadiusOfSphere(width() + 0.05, ballOffset());
}