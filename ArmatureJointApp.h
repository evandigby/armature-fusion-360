#pragma once

#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>
#include <CAM/CAMAll.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "ArmatureJoint/CommandCreated.h"

using namespace std;
using namespace adsk::core;
using namespace adsk::fusion;
using namespace adsk::cam;

class ArmatureJointApp {
public:
	ArmatureJointApp();
	~ArmatureJointApp();

private:
	Ptr<Application> app;
	Ptr<UserInterface> ui;
	Ptr<CommandDefinition> button;
	Ptr<CommandControl> control;
	ArmatureJoint::CommandCreated* commandCreatedEvent;
};