#pragma once

#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>

#include "ArmatureValues.h"

using namespace adsk::core;
using namespace adsk::fusion;

class ArmatureCommandExecuted : public CommandEventHandler {
private:
	Ptr<Application> app;
public:
	ArmatureCommandExecuted(Ptr<Application> _app) {
		app = _app;
	}

	bool createJointPlate(Ptr<Component> component, Ptr<ConstructionPlane> plane, shared_ptr<ArmatureValues> values);
	bool createJointBall(Ptr<Component> component, shared_ptr<ArmatureValues> values);
	void notify(const Ptr<CommandEventArgs>& eventArgs) override;
};
