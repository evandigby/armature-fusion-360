#include "ArmatureCommandInputChanged.h"
#include "ArmatureValues.h"

void ArmatureCommandInputChanged::notify(const Ptr<InputChangedEventArgs>& eventArgs) {
	if (!eventArgs)
		return;

	auto inputs = eventArgs->inputs();
	if (!inputs)
		return;

	auto values = ArmatureValues::create(eventArgs->inputs());
	if (!values)
		return;

	values->setExtents();
}
