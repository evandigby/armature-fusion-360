#pragma once

#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>

#include "Values.h"

using namespace adsk::core;
using namespace adsk::fusion;

namespace ArmatureJoint {
	class JointPlate
	{
	private:
		Ptr<Component> component;
		Ptr<ConstructionPlane> plane;
		shared_ptr<Values> values;
		bool top;
		Ptr<ExtrudeFeature> _extrude;
		Ptr<Sketch> _sketch;
		Ptr<BRepBody> _body;

		Ptr<Sketch> plateSketch();
		Ptr<ExtrudeFeature> plateExtrude();
		bool plateChamfer(Ptr<BRepBody> plateBody);
		bool plateFillet(Ptr<BRepBody> plateBody);


	public:
		static shared_ptr<JointPlate> create(Ptr<Component> _component, Ptr<ConstructionPlane> _plane, shared_ptr<Values> _values, bool _top);

		JointPlate(Ptr<Component> _component, Ptr<ConstructionPlane> _plane, shared_ptr<Values> _values, bool _top) {
			component = _component;
			plane = _plane;
			values = _values;
			top = _top;
		}

		Ptr<BRepBody> body();
	};
};