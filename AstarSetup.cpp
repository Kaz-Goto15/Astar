#include "AstarSetup.h"
#include <Windows.h>
AstarSetup::AstarSetup()
{
	GetPrivateProfileString("SCREEN", "Caption", "�@", *attributeStr[FLOOR].c_str(), 64, ".\\setup.ini");
}

AstarSetup::~AstarSetup()
{
}
