#include "ArmatureJointApp.h"

ArmatureJointApp* app;

extern "C" XI_EXPORT bool run(const char* context)
{
	app = new ArmatureJointApp();

	return true;
}

extern "C" XI_EXPORT bool stop(const char* context)
{
	if (app)
		delete app;

	return true;
}


#ifdef XI_WIN

#include <windows.h>

BOOL APIENTRY DllMain(HMODULE hmodule, DWORD reason, LPVOID reserved)
{
	switch (reason)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

#endif // XI_WIN
