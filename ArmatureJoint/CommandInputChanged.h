#pragma once

#include <Core/CoreAll.h>
#include <Fusion/FusionAll.h>

using namespace adsk::core;
using namespace adsk::fusion;

namespace ArmatureJoint {
	class CommandInputChanged : public InputChangedEventHandler {
		void notify(const Ptr<InputChangedEventArgs>& eventArgs) override;
	};
}