#include "AstarSetup.h"
#include <Windows.h>
AstarSetup::AstarSetup()
{
	GetPrivateProfileString("SCREEN", "Caption", "Å@", *attributeStr[FLOOR].c_str(), 64, ".\\setup.ini");
}

AstarSetup::~AstarSetup()
{
}
