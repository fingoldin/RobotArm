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

	Robot robot(TEXT("COM4"), CBR_115200, 45.3, 28.1, 8.0, 1.1);

	const int r_buffer_len = 256;
	char r_buffer[r_buffer_len];

	// reset
	//robot.moveTo(5.0, 30.0, 0.0, 0.0);
	robot.moveToAngles(M_PI / 2.0, M_PI / 2.0, 0.6, M_PI, 0.0);
	
	while (robot.isValid())
	{
		XINPUT_STATE state = xbox.getRawState();

		double dx = 0.0,
			dpan = 0.0,
			dz = 0.0,
			sens = 0.02;

		dx = sens * Xbox360::normalizeJoystickValue(state.Gamepad.sThumbLY);
		dpan = sens * Xbox360::normalizeJoystickValue(state.Gamepad.sThumbLX);
		dz = sens * Xbox360::normalizeJoystickValue(state.Gamepad.sThumbRY);

		//robot.move(dx, dz, dpan, 0.0);
		
		robot.moveAngles(dpan, dx, dz, 0.0, 0.0);

		// Recieved string ends in "\n"
		DWORD lr = robot.receive(r_buffer, r_buffer_len);
		if (lr > 1)
			printf("Received: %.*s\n", lr-1, r_buffer);

		robot.flush();

		Sleep(15);
	}

    return 0;
}

