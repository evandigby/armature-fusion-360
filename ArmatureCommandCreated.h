#pragma once

#include "ArmatureCommandExecuted.h"
#include "ArmatureCommandInputChanged.h"

class ArmatureCommandCreated : public CommandCreatedEventHandler {
private:
	Ptr<Application> app;
	unique_ptr<ArmatureCommandExecuted> _onExecute;
	unique_ptr<ArmatureCommandInputChanged> _onInputChanged;

public:
	ArmatureCommandCreated(Ptr<Application> _app) {
		app = _app;
		_onExecute = unique_ptr<ArmatureCommandExecuted>(new ArmatureCommandExecuted(app));
		_onInputChanged = unique_ptr<ArmatureCommandInputChanged>(new ArmatureCommandInputChanged());
	}

	void notify(const Ptr<CommandCreatedEventArgs>& eventArgs) override;
};