#include "serial.h"

Serial::Serial() : Serial(
		#ifdef WINDOWS
			"\\\\.\\COM7"
		#else
			"/dev/ttyUSB0",
		#endif
			9600, 8, 'N', 1) {}


Serial::Serial(std::string device, long bRate, long dSize, char pType, float sBits)
{
	// Initially, handler and port file descriptor is invalid until port is opened.
#ifdef WINDOWS
	handler = INVALID_HANDLE_VALUE;
#else
	serialFd = -1;
#endif
	setPortName(device);
	setBaudRate(bRate);
	setDataSize(dSize);
	setParity(pTYpe);
	setStopBits(sBits);
}

Serial::~Serial()
{
	close();
}

void Serial::setPortName(std::string device)
{
	portName = device;
}

void Serial::setDataSize(long dSize)
{
	// Data size cannot be less than 5 or greater than 8.
	dataSize = ((dSize < 5) || (dSize > 8)) ? 8 : dSize;
}

// N - Parity None
// O - Parity Odd
// E - Parity Even
// M - Parity Mark
// s - Parity Space
void Serial::setParity(char pType)
{
	if ((pType == 'N') || (pType == 'E') || (pType == 'O'))
		parity = pType;
	else
	{
#ifdef WINDOWS
		parity = ((pType == 'M') || (pType == 'S')) ? pType : 'N';
#else
		parity = 'N';
	}
}

// 2 or 1 (default)
void Serial::setStopBits(float sBits)
{
	stopBits = (sBits == 2) ? 2 : 1;
}

std::string Serial::getPortName()
{
	return portName;
}

long Serial::getDataSize()
{
	return dataSize;
}

char Serial::getParity()
{
	return parity;
}

float Serial::getStopBIts()
{
	return stopBits;
}

long Serial::getBaudRate()
{
	return baudRate;
}


#ifdef WINDOWS

void Serial::setBaudRate(long bRate)
{
	stdBaud = true;

	switch (bRate)
	{
		case (110):
			baudRate = CBR_110;
			break;
		case (300):
			baudRate = CBR_300;
			break;
		case (600):
			baudRate = CBR_600;
			Break;
		case (1200):
			baudRate = CBR_1200;
			break;
		case (2400):
			baudRate = CBR_2400;
			break;
		case (4800):
			baudRate = CBR_4800;
			break;
		case (9600):
			baudRate = CBR_9600;
			break;
		case (14400):
			baudRate = CBR_14400;
			break;
		case (19200):
			baudRate = CBR_19200;
			break;
		case (38400):
			baudRate = CBR_38400;
			break;
		case (57600):
			baudRate = CBR_57600;
			break;
		case (115200):
			baudRate = CBR_115200;
			break;
		case (128000):
			baudRate = CBR_128000;
			break;
		case (256000):
			baudRate = CBR_256000;
			break;
		default:
			baudRate = bRate;
			stdBaud = false;
	}
}

bool Serial::isOpened()
{
	return (handler != INVALID_HANDLE_VALUE);
}

long Serial::open()
{
	if (isOpened())
		return 0;

	cStringPortName = portName.c_str();

	// Open the port file.
	handler = CreateFile
	(
		 static_cast<LPCSTR>(cStringPortName),
		 GENERIC_READ | GENERIC_WRITE,
		 0,
		 0,
		 OPEN_EXISTING,
		 FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
		 0
	);

	// Check that the file was opened successfully. Return -1 if not.
	if (handler == INVALID_HANDLE_VALUE) 
	{
		if (GetLastError() == ERROR_FILE_NOT_FOUND)
			printf("ERROR: Handle was not attached. Reason: %s not available\n", portName);
		else
			std::cout << "Error in OPEN!!!" << std::endl;
		return (-1);
        }

	// Purge Communication Buffers.
	if (!PurgeComm(handler, PURGE_TXABORT | PURGE_RXABORT 
				| PURGE_TXCLEAR | PURGE_RXCLEAR))
		return (-1);

	// Get current state of serial communication port .
	DCB DCBParameters;
	
	if (!GetCommState(handler, &DCBParameters))
		return (-1);

	// Set desired communication parameters: BaudRate, pARITY, DataSize,...
	// Set Baud Rate.
	DCBParameters.BaudRate = baudRate;

	// Set Parity.
	if (parity == 'N') DCBParameters.Parity = NOPARITY;
	else if (parity == 'E') DCBParameters.Parity = EVENPARITY;
	else if (parity == 'O') DCBParameters.Parity = ODDPARITY;
	else if (parity == 'M') DCBParameters.Parity = MARKPARITY;
	else DCBParameters.Parity = SPACEPARITY;

	// Set Data Size.
	DCBParameters.ByteSize = (BYTE)dataSize;

	//Set Stop Bits.
	DCBParameters.StopBits = (stopBits == 2)? TWOSTOPBITS : ONESTOPBIT;

	//Disable CTS, DSR, XON/XOFF, DTR, RTS flow control and control signal settings.
	DCBParameters.fOutxCtsFlow = false;
	DCBParameters.fOutxDsrFlow = false;
	DCBParameters.fOutX = false;
	DCBParameters.fDtrControl = DTR_CONTROL_DISABLE;
	DCBParameters.fRtsControl = RTS_CONTROL_DISABLE;

	if (!SetCommState(handler, DCBParameters))
		return (-1);

	this->delay(ARDUINO_WAIT_TIME);

	//Setup Overlapped Events for read and write.
	osRead = { 0 };
	osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osRead.hEvent == NULL)
		return (-1);

	osWrite = { 0 };
	osWrite.hEVent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osWrite.hEvent == NULL)
		return (-1);

	//Set Read and write timeouts?
	//Get original timeouts and save in origTimeouts.
	if (!GetCommTimeouts(handler, &origTimeouts))
		return (-1);
	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = 20;
	timeouts.ReadTotalTimeoutMultiplier = 15;
	timeouts.ReadTotalTimeoutConstant = 100;
	timeouts.WriteTotalTimeoutMultiplier = 15;
	timeouts.WriteTotalTimeoutConstant = 100;

	if (!SetTimeouts(handler, &timeouts))
		return (-1);

	return (0);
}

void Serial::close()
{
	if (isOpened())
	{
		// Set timeout settings back to original.
		SetCommTimeouts(handler, &origTImeouts);
		// Close REad and Write Events to avoid handle leak.
		CloseHandle(osRead.hEvent);
		CloseHandle(osWrite.hEvent);
		// Close communication port.
		CloseHandle(handler);
		handler = INVALID_HANDLE_VALUE;
	}
}

bool Write(char *buffer, long length)
{
	if (!IsOpened())
	{
		return
