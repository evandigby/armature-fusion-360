#pragma once

#include <Core/CoreAll.h>
#include <list>

using namespace std;
using namespace adsk::core;

namespace ArmatureJoint {
	class Values {
	public:
		static double circleRadiusOfSphere(double sphereDiameter, double offset);
		static double diameterForCircleRadiusOfSphere(double circleRadius, double offset);
		static double defaultLength();
		static double defaultWidth();
		static double defaultThickness();
		static double defaultBallDiameter();
		static double defaultBoltHoleDiameter();
		static double defaultRows();
		static double defaultCols();
		static double defaultHoleDiameter();
		static shared_ptr<Values> create(Ptr<CommandInputs> inputs);
		double ballDiameter();
		double width();
		double length();
		double thickness();
		void setExtents();
		double ballOffset();
		double plateOffset();
		double ballRadius();
		double ballX(int col);
		double ballY(int row);
		double ballZ();
		double circleRadius();
		double circleArea();
		double circleCircumference();
		double minWidth();
		double maxBallDiameter();
		double holeDiameter(int row, int col);
		double boltHoleDiameter();
		double boltHoleRadius();
		double boltCircleArea();
		double holeRadius(int row, int col);
		double chamferLength();
		double chamferAngle();
		double expectedArea();
		
		int numJointTypes(std::string jointType);
		int rows();
		int cols();
		std::string jointType(int row, int col);
		std::string name();

		static Ptr<UnitsManager> unitsManager;

	private:
		Ptr<StringValueCommandInput> nameInput;
		Ptr<DistanceValueCommandInput> lengthInput;
		Ptr<DistanceValueCommandInput> widthInput;
		Ptr<DistanceValueCommandInput> thicknessInput;
		Ptr<DistanceValueCommandInput> ballDiameterInput;
		Ptr<IntegerSpinnerCommandInput> rowsInput;
		Ptr<IntegerSpinnerCommandInput> colsInput;
		Ptr<TableCommandInput> tableInput;
		Ptr<DistanceValueCommandInput> boltHoleInput;
	};
}