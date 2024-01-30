#pragma once
#include <string>

enum NODE_ATTRIBUTE {
	FLOOR,
	WALL,
	PATH,
	START,
	END,
	MAX
};

class AstarSetup
{
public:
	AstarSetup();
	~AstarSetup();
protected:
	std::string attributeStr[MAX];
};

