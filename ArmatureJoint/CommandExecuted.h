#pragma once

#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>

#include "Values.h"

using namespace adsk::core;
using namespace adsk::fusion;

namespace ArmatureJoint {
	class CommandExecuted : public CommandEventHandler {
	private:
		Ptr<Application> app;
	public:
		CommandExecuted(Ptr<Application> _app) {
			app = _app;
		}

		bool createJointPlate(Ptr<Component> component, Ptr<ConstructionPlane> plane, shared_ptr<Values> values);
		bool createJointBall(Ptr<Component> component, shared_ptr<Values> values);
		void notify(const Ptr<CommandEventArgs>& eventArgs) override;
	};
}
