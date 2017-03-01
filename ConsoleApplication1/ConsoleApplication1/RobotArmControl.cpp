// RobotArmControl.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "Robot.h"
#include "Xbox360.h"

int main(int argc, char ** argv)
{
	DWORD port = Xbox360::getOpenController(4);
	if(port == -1)
	{
		printf("Could not find any controllers!\n");
		return 1;
	}
	
	Xbox360 xbox(port);

	Robot robot(TEXT("COM3"), CBR_115200, 35.0, 22.5, 8.0, 0.7, 11.3);

	const int r_buffer_len = 256;
	char r_buffer[r_buffer_len];

	robot.flush();

	// reset
	//robot.moveTo(15.0, 15.0, 3.0 * M_PI / 4.0, 0.0, 0.0, 0);
	//robot.moveToAngles(2.44346, 1.719375, 0.0, -0.06, 0.0, 0);
	//robot.flush();

	//Sleep(500);

	// 5.1696, 26.5641

	robot.moveTo(25.0, 10.0, 2.44346, 0.0, 0.0, 0);
	robot.flush();

	bool reset_button = false;

	while (robot.isValid())
	{
		XINPUT_STATE state = xbox.getRawState();

		double dx = 0.0,
			dpan = 0.0,
			dwrist = 0.0,
			dz = 0.0,
			dhand = 0.0,
			sens = 0.2;

		int claw = 0;

		dz = sens * Xbox360::normalizeJoystickValue(state.Gamepad.sThumbLY);
		dpan = sens * 0.02 * Xbox360::normalizeJoystickValue(state.Gamepad.sThumbLX);
		dx = sens * Xbox360::normalizeJoystickValue(state.Gamepad.sThumbRY);

		if (state.Gamepad.wButtons & XINPUT_GAMEPAD_X)
			dhand += 0.1 * sens;
		if (state.Gamepad.wButtons & XINPUT_GAMEPAD_B)
			dhand -= 0.1 * sens;

		if (state.Gamepad.wButtons & XINPUT_GAMEPAD_Y)
			dwrist += 0.1 * sens;
		if (state.Gamepad.wButtons & XINPUT_GAMEPAD_A)
			dwrist -= 0.1 * sens;

		if (state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_LEFT)
			claw = 1;
		else if (state.Gamepad.wButtons & XINPUT_GAMEPAD_DPAD_RIGHT)
			claw = 2;

		//if(h > 0.0)
		robot.move(dz, dx, dpan, dwrist, dhand, claw);
		
		//if(!(state.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_THUMB) && reset_button)
		//	robot.moveToAngles(2.44346, 1.719375, 0.0, -0.06, 0.0, 0);

		reset_button = (bool)(state.Gamepad.wButtons & XINPUT_GAMEPAD_RIGHT_THUMB);

		//robot.moveAngles(dpan, 0.1*dx, 0.1*dz, dwrist, dhand, claw);

		// Recieved string ends in "\n"
		//DWORD lr = robot.receive(r_buffer, r_buffer_len);
		//if (lr > 1)
		//	printf("Received: %.*s\n", lr-1, r_buffer);

		robot.flush();

		Sleep(15);
	}

	while (1)
		;
    return 0;
}

