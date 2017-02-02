#pragma once

class Robot
{
public:
	Robot(LPCTSTR portName, DWORD baud, double A, double B, double C, double D);
	~Robot();

	BOOL send(char * buffer, DWORD buffer_len);
	DWORD receive(char * buffer, DWORD buffer_size);

	BOOL moveTo(double z, double x, double pan, double hand);
	BOOL move(double dz, double dx, double dpan, double dhand);
	
	// For testing purposes
	BOOL moveToAngles(double pan, double tilt, double elbow, double wrist, double hand);
	BOOL moveAngles(double dpan, double dtilt, double delbow, double dwrist, double dhand);

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

	// Angles
	double Pan_A;
	double Tilt_A;
	double Elbow_A;
	double Wrist_A;
	double Hand_A;

	HANDLE handle;
	LPCTSTR portName;
	DWORD baud;

	BOOL valid;
};

