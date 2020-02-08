#include "CommandInputChanged.h"
#include "Values.h"

namespace ArmatureJoint {
	void CommandInputChanged::notify(const Ptr<InputChangedEventArgs>& eventArgs) {
		if (!eventArgs)
			return;

		auto inputs = eventArgs->inputs();
		if (!inputs)
			return;

		auto values = Values::create(eventArgs->inputs());
		if (!values)
			return;

		values->setExtents();
	}
}