#include "stdafx.h"
#include "Xbox360.h"


Xbox360::Xbox360(DWORD port)
{
	this->port = port;

	this->refreshState();
}

Xbox360::~Xbox360()
{
}


void Xbox360::refreshState(void)
{
	this->clearState();

	DWORD res = XInputGetState(this->port, &this->state);
	if (res != ERROR_SUCCESS)
	{
		fprintf(stderr, "Could not access Xbox controller on port %d!\n", this->port);
	}
}

XINPUT_STATE Xbox360::getRawState(void)
{
	this->refreshState();

	return this->state;
}

void Xbox360::clearState(void)
{
	memset(&this->state, 0, sizeof(XINPUT_STATE));
}

double Xbox360::normalizeJoystickValue(SHORT val)
{
	if (val > XINPUT_GAMEPAD_LOWER_THRESHOLD || val < -XINPUT_GAMEPAD_LOWER_THRESHOLD)
		return (double)(val - XINPUT_GAMEPAD_LOWER_THRESHOLD) / (double)(32767 - XINPUT_GAMEPAD_LOWER_THRESHOLD);
	else
		return 0.0;
}


DWORD Xbox360::getOpenController(DWORD max_port)
{
	XINPUT_STATE tstate;
	DWORD port = -1;

	for(DWORD i = 0; i < max_port; i++)
	{
		DWORD res = XInputGetState(i, &tstate);
		if(res == ERROR_SUCCESS)
		{
			printf("Xbox controller on port %d is ON!\n", i + 1);
			port = i;
		}
	}

	return port;
}