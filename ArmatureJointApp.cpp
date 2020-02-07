#include "ArmatureJointApp.h"

ArmatureJointApp::ArmatureJointApp() {
	auto app = Application::get();
	assert(app);
	
	auto ui = app->userInterface();
	assert(ui);

	auto definitions = ui->commandDefinitions();
	assert(definitions);

	button = definitions->addButtonDefinition(
		ARMATURE_COMMAND_ID,
		"Create Armature",
		"Creates a stop motion animation armature",
		""
	);
	if (!button) {
		return;
	}

	auto commandCreated = button->commandCreated();
	assert(commandCreated);

	commandCreatedEvent = new ArmatureCommandCreated(app);
	commandCreated->add(commandCreatedEvent);

	auto panels = ui->allToolbarPanels();
	assert(panels);

	auto panel = panels->itemById("SolidScriptsAddinsPanel");
	assert(panel);

	auto controls = panel->controls();
	assert(controls);

	control = controls->addCommand(button);
}

ArmatureJointApp::~ArmatureJointApp() {
	if (commandCreatedEvent) {
		delete commandCreatedEvent;
		commandCreatedEvent = nullptr;
	}

	if (control) {
		control->deleteMe();
		control = nullptr;
	}

	if (button) {
		button->deleteMe();
		button = nullptr;
	}

	if (ui)
		ui = nullptr;

}