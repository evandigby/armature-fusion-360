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

		bool createJointBall(Ptr<Component> component, shared_ptr<Values> values);
		bool createJointNuts(Ptr<Component> component, shared_ptr<Values> values);

		void notify(const Ptr<CommandEventArgs>& eventArgs) override;
	};
}
