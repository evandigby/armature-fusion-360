#pragma once

#include <Core/CoreAll.h>

using namespace std;
using namespace adsk::core;

namespace ArmatureJoint {
	class Values {
	public:
		static double circleRadiusOfSphere(double sphereDiameter, double offset);
		static double diameterForCircleRadiusOfSphere(double circleRadius, double offset);
		static double defaultLength(Ptr<UnitsManager> unitsManager);
		static double defaultWidth(Ptr<UnitsManager> unitsManager);
		static double defaultThickness(Ptr<UnitsManager> unitsManager);
		static double defaultBallDiameter(Ptr<UnitsManager> unitsManager);
		static double defaultRows(Ptr<UnitsManager> unitsManager);
		static double defaultCols(Ptr<UnitsManager> unitsManager);
		static shared_ptr<Values> create(Ptr<CommandInputs> inputs);
		double ballDiameter();
		double width();
		double length();
		double thickness();
		void setExtents();
		double ballOffset();
		double ballRadius();
		double ballX(int col);
		double ballY(int row);
		double ballZ();
		double circleRadius();
		double circleArea();
		double minWidth();
		double maxBallDiameter();
		int rows();
		int cols();

	private:
		Ptr<DistanceValueCommandInput> lengthInput;
		Ptr<DistanceValueCommandInput> widthInput;
		Ptr<DistanceValueCommandInput> thicknessInput;
		Ptr<DistanceValueCommandInput> ballDiameterInput;
		Ptr<IntegerSpinnerCommandInput> rowsInput;
		Ptr<IntegerSpinnerCommandInput> colsInput;
	};
}