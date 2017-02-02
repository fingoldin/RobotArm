#include "stdafx.h"
#include "Robot.h"


Robot::Robot(LPCTSTR portName, DWORD baud, double A, double B, double C, double D)
{
	this->portName = portName;
	this->baud = baud;
	this->A = A;
	this->B = B;
	this->C = C;
	this->D = D;

	this->valid = this->UART_Init();

	if (!this->valid)
	{
		wprintf(L"Could not initialize robot on port %ws", this->portName);
	}
}

Robot::~Robot()
{
	if (!this->UART_Close())
	{
		wprintf(L"Could not close connection to robot on port %ws", this->portName);
	}
}


BOOL Robot::UART_Init()
{
	DCB portDCB;	      // _DCB struct for serial configuration
	BOOL result = FALSE;  // Return value
	COMMTIMEOUTS comTOUT; // Communication timeout
	this->handle = CreateFile(this->portName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_FLAG_WRITE_THROUGH, NULL);
	// Try opening port communication
	if (this->handle == INVALID_HANDLE_VALUE)
	{
		printf("ERROR: Cannot open connection to robot on port %ws\n", this->portName);
		return FALSE;
	}
	// NEW SETTINGS
	portDCB.DCBlength = sizeof(DCB);// Setup config length
	GetCommState(this->handle, &portDCB);	// Get default port state
	portDCB.BaudRate = this->baud;	// Set baud rate
	portDCB.fBinary = TRUE;		// Enable Binary mode
	portDCB.fParity = FALSE;	// Disable parity 
	portDCB.fOutxCtsFlow = FALSE;	// No CTS 
	portDCB.fOutxDsrFlow = FALSE;	// No DSR
	portDCB.fDtrControl = DTR_CONTROL_DISABLE;	// No DTR
	portDCB.fDsrSensitivity = FALSE;	// No DSR sensitivity 
	portDCB.fTXContinueOnXoff = TRUE;	// TX on XOFF
	portDCB.fOutX = FALSE;			// No XON/XOFF
	portDCB.fInX = FALSE;			//
	portDCB.fErrorChar = FALSE;		// No error correction
	portDCB.fNull = FALSE;			// Keep NULL values
	portDCB.fRtsControl = RTS_CONTROL_DISABLE; // Disable RTS
	portDCB.fAbortOnError = FALSE;	// Disable abort-on-error
	portDCB.ByteSize = 8;			// 8-bit frames
	portDCB.Parity = NOPARITY;		// Parity: none
	portDCB.StopBits = ONESTOPBIT;		// StopBits: 1

										// Try reconfiguring COM port
	if (!SetCommState(this->handle, &portDCB))
	{
		wprintf(L"ERROR: Cannot configure robot on port %ws\n", this->portName);
		return FALSE;
	}

	/// Communication timeout values
	result = GetCommTimeouts(this->handle, &comTOUT);
	comTOUT.ReadIntervalTimeout = 10;
	comTOUT.ReadTotalTimeoutMultiplier = 1;
	comTOUT.ReadTotalTimeoutConstant = 1;
	/// Set new timeout values
	result = SetCommTimeouts(this->handle, &comTOUT);
	return TRUE;
}

BOOL Robot::UART_Close(void)
{
	if (this->handle == NULL) return FALSE;
	CloseHandle(this->handle);
	this->handle = NULL;
	return TRUE;
}


BOOL Robot::send(char * buffer, DWORD buffer_len)
{
	DWORD bytesTransmitted;
	if (!WriteFile(this->handle, buffer, buffer_len, &bytesTransmitted, NULL))
	{
		DWORD Errors;
		COMSTAT Status;
		ClearCommError(buffer, &Errors, &Status);
		wprintf(L"ERROR: Unable to send data to robot on port %ws\n", this->portName);
		return FALSE;
	}
	else
	{
		return TRUE;
	}
}

DWORD Robot::receive(char * buffer, DWORD buffer_size)
{
	DWORD bytesTransmitted = 0;		// Byte counter
	DWORD status = EV_RXCHAR;		// transmission status mask

	memset(buffer, 0, buffer_size);		// Clear input buffer

	SetCommMask(this->handle, EV_RXCHAR);		// Set up event mask
	WaitCommEvent(this->handle, &status, 0);	// Listen for RX event

	if (status & EV_RXCHAR)			// If event occured
	{
		DWORD success = 0;
		char c = 0;
		do
		{
			if (!ReadFile(this->handle, &c, 1, &success, NULL))	// Read 1 char
			{
				// If error occured, print the message and exit
				DWORD Errors;
				COMSTAT Status;
				ClearCommError(this->handle, &Errors, &Status);	// Clear errors
				memset(buffer, 0, buffer_size);		// Clear input buffer
				wprintf(L"ERROR: Unable to receive data from robot on port %ws\n", this->portName); // Print error message
				return -1;
			}
			else
			{
				buffer[bytesTransmitted] = c;	// Add last character
				bytesTransmitted++;		// Increase trans. counter
			}
		} while ((success == 1) && (c != '\n'));		// do until the end of message
	}

	return bytesTransmitted;
}


BOOL Robot::moveTo(double z, double x, double pan, double hand)
{
	this->Z = max(0.0, min(this->A + this->B, z));
	this->X = max(0.0, min(this->A + this->B, x));

	this->Pan_A = max(0.0, min(2.0*M_PI, pan));
	//this->Wrist_A = max(0.0, min(2*M_PI, wrist));
	this->Hand_A = max(0.0, min(2.0*M_PI, hand));

	this->updateAngles();

	return this->sendAngles();
}

BOOL Robot::move(double dz, double dx, double dpan, double dhand)
{
	return this->moveTo(this->Z + dz, this->X + dx, this->Pan_A + dpan, this->Hand_A + dhand);
}

// For testing purposes
BOOL Robot::moveToAngles(double pan, double tilt, double elbow, double wrist, double hand)
{
	this->Pan_A = max(0.0, min(2.0*M_PI, pan));
	this->Tilt_A = max(0.0, min(2.0*M_PI, tilt));
	this->Elbow_A = max(0.0, min(2.0*M_PI, elbow));
	this->Wrist_A = max(0.0, min(2.0*M_PI, wrist));
	this->Hand_A = max(0.0, min(2.0*M_PI, hand));

	this->updateCoords();

	return this->sendAngles();
}

// For testing purposes
BOOL Robot::moveAngles(double dpan, double dtilt, double delbow, double dwrist, double dhand)
{
	return this->moveToAngles(this->Pan_A + dpan, this->Tilt_A + dtilt, this->Elbow_A + delbow, this->Wrist_A + dwrist, this->Hand_A + dhand);
}

BOOL Robot::sendAngles()
{
	char buffer[11];

	USHORT pan = (USHORT)(10000.0 * this->Pan_A / (2.0*M_PI));
	buffer[0] = (char)(pan / 100 + 20);
	buffer[1] = (char)(pan % 100 + 20);

	USHORT tilt = (USHORT)(10000.0 * this->Tilt_A / (2.0*M_PI));
	buffer[2] = (char)(tilt / 100 + 20);
	buffer[3] = (char)(tilt % 100 + 20);

	USHORT elbow = (USHORT)(10000.0 * this->Elbow_A / (2.0*M_PI));
	buffer[4] = (char)(elbow / 100 + 20);
	buffer[5] = (char)(elbow % 100 + 20);

	//("%d %d\n", buffer[4], buffer[5]);

	USHORT wrist = (USHORT)(10000.0 * this->Wrist_A / (2.0*M_PI));
	buffer[6] = (char)(wrist / 100 + 20);
	buffer[7] = (char)(wrist % 100 + 20);

	USHORT hand = (USHORT)(10000.0 * this->Hand_A / (2.0*M_PI));
	buffer[8] = (char)(hand / 100 + 20);
	buffer[9] = (char)(hand % 100 + 20);

	buffer[10] = '\n';

	return this->send(buffer, 11);
}

void Robot::updateAngles()
{
	double h = this->Z + this->C;
	double h2 = h*h;
	double h4 = h*h*h*h;
	double h6 = h*h*h*h*h*h;

	double a = this->A;
	double a2 = a*a;
	double a3 = a*a*a;
	double a4 = a*a*a*a;
	double a6 = a*a*a*a*a*a;

	double b = sqrt(this->B*this->B + this->D*this->D);
	double b2 = b*b;
	double b4 = b*b*b*b;

	double x = this->X;
	double x2 = x*x;
	double x3 = x*x*x;
	double x4 = x*x*x*x;

	double tilt_angle1 = acos((a3*x - sqrt(-a6 * h2 + 2.0*a4*b2*h2 + 2.0*a4*h4 + 2.0*a4*h2*x2 - a2*b4*h2 + 2.0*a2*b2*b4 + 2.0*a2*b2*h2*x2 - a2*h6 - 2.0*a2*h4*x2 - a2*h2*x4) - a*b2*x + a*h2*x + a*x3) / (2.0*(a2*h2 + a2*x2)));
	double elbow_angle1 = asin((x - a*cos(tilt_angle1)) / b);
	//double elbow_angle1 = acos(((a2*b2*x2) / (2 * h*(a2*h2 + a2*x2)) - (a2*h*x2) / (2 * (a2*h2 + a2*x2)) - (a2*x4) / (2 * h*(a2*h2 + a2*x2)) + a2 / (2 * h) - (a4*x2) / (2 * h*(a2*h2 + a2*x2)) + (a*x*sqrt(a6*(-h2) + 2 * a4*b2*h2 + 2 * a4*h4 + 2 * a4*h2*x2 - a2*b4*h2 + 2 * a2*b2*h4 + 2 * a2*b2*h2*x2 - a2*h6 - 2 * a2*h4*x2 - a2*h2*x4)) / (2 * h*(a2*h2 + a2*x2)) - b2 / (2 * h) + x2 / (2 * h) - h / 2) / b);

	// turn 2 into 2.0 for commented-out elbow_angle
	double tilt_angle2 = acos((a3*x + sqrt(-a6 * h2 + 2.0*a4*b2*h2 + 2.0*a4*h4 + 2.0*a4*h2*x2 - a2*b4*h2 + 2.0 * a2*b2*b4 + 2.0 * a2*b2*h2*x2 - a2*h6 - 2.0 * a2*h4*x2 - a2*h2*x4) - a*b2*x + a*h2*x + a*x3) / (2.0 * (a2*h2 + a2*x2)));
	//double elbow_angle2 = acos(((a2*b2*x2) / (2 * h*(a2*h2 + a2*x2)) - (a2*h*x2) / (2 * (a2*h2 + a2*x2)) - (a2*x4) / (2 * h*(a2*h2 + a2*x2)) + a2 / (2 * h) - (a4*x2) / (2 * h*(a2*h2 + a2*x2)) - (a*x*sqrt(a6*(-h2) + 2 * a4*b2*h2 + 2 * a4*h4 + 2 * a4*h2*x2 - a2*b4*h2 + 2 * a2*b2*h4 + 2 * a2*b2*h2*x2 - a2*h6 - 2 * a2*h4*x2 - a2*h2*x4)) / (2 * h*(a2*h2 + a2*x2)) - b2 / (2 * h) + x2 / (2 * h) - h / 2) / b);
	double elbow_angle2 = asin((x - a*cos(tilt_angle2)) / b);
	
	
	// Angles are sent relative to the arms (between the arms) and range from 0 rad to 2pi rad, and do not correspond to the actual angles of the servos

	double tilt = tilt_angle1;
	double elbow = M_PI / 2.0 - tilt_angle1 + elbow_angle1 + atan(this->D / this->B);
	double wrist = M_PI - elbow_angle1;

	if (!isnan(tilt))
		this->Tilt_A = max(0.0, min(2.0*M_PI, tilt));
	if (!isnan(elbow))
		this->Elbow_A = max(0.0, min(2.0*M_PI, elbow));
	if (!isnan(wrist))
		this->Wrist_A = max(0.0, min(2.0*M_PI, wrist));

	printf("updateAngles: Angles for position (%f, %f): %f, %f, %f\n", this->X, this->Z, this->Tilt_A, this->Elbow_A, this->Wrist_A);
}

void Robot::updateCoords()
{
	// Angles are stored from 0.0 to 2pi rad
	double at = atan(this->D / this->B);

	this->X = this->A*cos(this->Tilt_A) + this->B*sin(this->Elbow_A - M_PI / 2.0 + this->Tilt_A - at);
	this->Z = this->A*sin(this->Tilt_A) - this->B*cos(this->Elbow_A - M_PI / 2.0 + this->Tilt_A - at) - this->C;

	//printf("updateCoords: Angles for position (%f, %f): %f, %f, %f\n", this->X, this->Z, this->Tilt_A, this->Elbow_A, this->Wrist_A);
}