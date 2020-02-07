#pragma once

#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>

using namespace adsk::core;
using namespace adsk::fusion;

class ArmatureCommandInputChanged : public InputChangedEventHandler {
	void notify(const Ptr<InputChangedEventArgs>& eventArgs) override;
};