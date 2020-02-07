#pragma once

#include <Core/CoreAll.h>

using namespace std;
using namespace adsk::core;

class ArmatureValues {
public:
	static double circleRadiusOfSphere(double sphereDiameter, double offset);
	static double diameterForCircleRadiusOfSphere(double circleRadius, double offset);
	static double defaultLength(Ptr<UnitsManager> unitsManager);
	static double defaultWidth(Ptr<UnitsManager> unitsManager);
	static double defaultThickness(Ptr<UnitsManager> unitsManager);
	static double defaultBallDiameter(Ptr<UnitsManager> unitsManager);
	static shared_ptr<ArmatureValues> create(Ptr<CommandInputs> inputs);
	double ballDiameter();
	double width();
	double length();
	double thickness();
	void setExtents();
	double ballOffset();
	double ballRadius();
	double ballX();
	double ballY();
	double ballZ();
	double circleRadius();
	double circleArea();
	double minWidth();
	double maxBallDiameter();

private:
	Ptr<DistanceValueCommandInput> lengthInput;
	Ptr<DistanceValueCommandInput> widthInput;
	Ptr<DistanceValueCommandInput> thicknessInput;
	Ptr<DistanceValueCommandInput> ballDiameterInput;
};