#pragma once

class Robot
{
public:
	Robot(LPCTSTR portName, DWORD baud, double A, double B, double C, double D, double H);
	~Robot();

	BOOL send(char * buffer, DWORD buffer_len);
	DWORD receive(char * buffer, DWORD buffer_size);

	BOOL moveTo(double z, double x, double pan, double wrist, double hand, int claw);
	BOOL move(double dz, double dx, double dpan, double dwrist, double dhand, int claw);
	
	// For testing purposes
	BOOL moveToAngles(double pan, double tilt, double elbow, double wrist, double hand, int claw);
	BOOL moveAngles(double dpan, double dtilt, double delbow, double dwrist, double dhand, int claw);

	BOOL isValid(void) { return !(!this->valid || this->handle == INVALID_HANDLE_VALUE); }

	void flush(void) { PurgeComm(this->handle, PURGE_RXCLEAR | PURGE_TXCLEAR); }

private:

	void updateAngles(void);
	void updateCoords(void);

	BOOL sendAngles(void);

	BOOL UART_Init(void);
	BOOL UART_Close(void);


	// Coords
	double X;
	double Z;

	// Arm lengths
	double A;
	double B;
	double C;
	double D;
	double H;

	// Angles
	double Pan_A;
	double Tilt_A;
	double Elbow_A;
	double Wrist_A;
	double Hand_A;
	double rel_wrist;

	// 0 is off, 1 is forward, 2 is backwards
	int claw_state;

	HANDLE handle;
	LPCTSTR portName;
	DWORD baud;

	BOOL valid;
};

