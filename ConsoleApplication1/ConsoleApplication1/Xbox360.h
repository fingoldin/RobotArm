#pragma once

class Xbox360
{
public:

	Xbox360(DWORD port);

	~Xbox360();
	

	void refreshState(void);
	XINPUT_STATE getRawState(void);
	void clearState(void);


	static DWORD getOpenController(DWORD max_port);

	static double normalizeJoystickValue(SHORT val);

private:

	DWORD port;
	XINPUT_STATE state;
};

