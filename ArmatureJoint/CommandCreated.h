#pragma once

#include "CommandExecuted.h"
#include "CommandInputChanged.h"

namespace ArmatureJoint {
	class CommandCreated : public CommandCreatedEventHandler {
	private:
		Ptr<Application> app;
		unique_ptr<CommandExecuted> _onExecute;
		unique_ptr<CommandInputChanged> _onInputChanged;

	public:
		CommandCreated(Ptr<Application> _app) {
			app = _app;
			_onExecute = unique_ptr<CommandExecuted>(new CommandExecuted(app));
			_onInputChanged = unique_ptr<CommandInputChanged>(new CommandInputChanged());
		}

		void notify(const Ptr<CommandCreatedEventArgs>& eventArgs) override;
	};
}