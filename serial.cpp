#include "serial.h"
#include <filesystem>

Serial::Serial() : Serial(
#ifdef WINDOWS
					   "\\\\.\\COM7",
#else
					   "/dev/ttyUSB0",
#endif
					   9600, 8, 'N', 1)
{
}

/**
 * Creates an instance of the Serial Class for serial communication.
 * Sets the configs accordingly.
 * @device: The portname.
 * @bRate: The baud rate.
 * @dSize: The data size.
 * @pType: THe parity type.
 * @sBits: The number of stop bits.
 */
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
	setParity(pType);
	setStopBits(sBits);
}

/**
 * Closes the serial communication port.
 */
Serial::~Serial()
{
	Close();
}

/**
 * setPortName - Sets the portName variable.
 * @device: The string to be set as the port name.
 * Returns: void.
 */
void Serial::setPortName(std::string device)
{
	portName = device;
}

/**
 * setDataSize - Sets the dataSize variable.
 * @dSize: The value to be set as the data size.
 * Returns: void.
 */
void Serial::setDataSize(long dSize)
{
	// Data size cannot be less than 5 or greater than 8.
	dataSize = ((dSize < 5) || (dSize > 8)) ? 8 : dSize;
}

/**
 * setParity - Sets the parity variable. Could be:
 * N - Parity None, O - Parity Odd, E - Parity Even,
 * M - Parity Mark, or S - Parity Space.
 * @pType; The character to be set as the parity type.
 * Returns: void.
 */
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
#endif
	}
}

/**
 * setStopBits: Sets the stopBIts variable. Could be 2 or 1 (default). Maybe 1.5? for Windows
 * @sBits: The float to set as the stop bits.
 * Returns: void.
 */
void Serial::setStopBits(float sBits)
{
	stopBits = (sBits == 2) ? 2 : 1;
}

/**
 * getPortName: Returns the portName used in communication.
 * Returns: portName.
 */
std::string Serial::getPortName()
{
	return portName;
}

/**
 * getDataSize - Returns the dataSize in communication.
 * Returns: dataSize.
 */
long Serial::getDataSize()
{
	return dataSize;
}

/**
 * getParity - Returns the parity used in communication.
 * Returns: parity.
 */
char Serial::getParity()
{
	return parity;
}

float Serial::getStopBIts()
{
	return stopBits;
}

/**
 * getBaudRate: Returns the baud rate used in communication.
 * Returns: baudRate.
 */
long Serial::getBaudRate()
{
	return baudRate;
}

// Definition of some functions that are specific to Windows OS.
#ifdef WINDOWS

/**
 * setBaudRate - Sets baudRate variable. 
 * Also sets stdBaud to true if bRate is one of the standard ones. Otherwise false.
 * Returns: void.
 */
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
		break;
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

/**
 * isOpened - Checks if communication port is open.
 * Returns: true, if the port is open. Otherwise false.
 */
bool Serial::isOpened()
{
	return (handler != INVALID_HANDLE_VALUE);
}

/**
 * Open - Opens comm. port; Sets comm. parameters; Creates read/write overlapped events; and Sets timeouts.
 * Returns: 0 on success, otherwise -1.
 */
int Serial::Open()
{
	if (isOpened())
		return 0;

#ifdef UNICODE
	std::wstring wtext(portName.begin(), portName.end());
#else
	std::string wtext = portName;

#endif

	// Open the port file.
	handler = CreateFile(
		wtext.c_str(),
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED,
		0);

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
	if (!PurgeComm(handler, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR))
		return (-1);

	// Get current state of serial communication port .
	DCB DCBParameters;

	if (!GetCommState(handler, &DCBParameters))
		return (-1);

	// Set desired communication parameters: BaudRate, pARITY, DataSize,...
	// Set Baud Rate.
	DCBParameters.BaudRate = baudRate;

	// Set Parity.
	if (parity == 'N')
		DCBParameters.Parity = NOPARITY;
	else if (parity == 'E')
		DCBParameters.Parity = EVENPARITY;
	else if (parity == 'O')
		DCBParameters.Parity = ODDPARITY;
	else if (parity == 'M')
		DCBParameters.Parity = MARKPARITY;
	else
		DCBParameters.Parity = SPACEPARITY;

	// Set Data Size.
	DCBParameters.ByteSize = (BYTE)dataSize;

	// Set Stop Bits.
	DCBParameters.StopBits = (stopBits == 2) ? TWOSTOPBITS : ONESTOPBIT;

	// Disable CTS, DSR, XON/XOFF, DTR, RTS flow control and control signal settings.
	DCBParameters.fOutxCtsFlow = false;
	DCBParameters.fOutxDsrFlow = false;
	DCBParameters.fOutX = false;
	DCBParameters.fDtrControl = DTR_CONTROL_DISABLE;
	DCBParameters.fRtsControl = RTS_CONTROL_DISABLE;

	if (!SetCommState(handler, &DCBParameters))
		return (-1);

	this->Delay(ARDUINO_WAIT_TIME);

	// Setup Overlapped Events for read and write.
	osRead = {0};
	osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osRead.hEvent == NULL)
		return (-1);
	fWaitOnRead = FALSE;

	osWrite = {0};
	osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osWrite.hEvent == NULL)
		return (-1);

	// Set Read and write timeouts?
	// Get original timeouts and save in origTimeouts.
	if (!GetCommTimeouts(handler, &origTimeouts))
		return (-1);
	COMMTIMEOUTS timeouts;
	timeouts.ReadIntervalTimeout = 20;
	timeouts.ReadTotalTimeoutMultiplier = 15;
	timeouts.ReadTotalTimeoutConstant = 100;
	timeouts.WriteTotalTimeoutMultiplier = 15;
	timeouts.WriteTotalTimeoutConstant = 100;

	if (!SetCommTimeouts(handler, &timeouts))
		return (-1);

	return (0);
}

/**
 * Close - Closes communication port; Reverts timeout settings; and Closes Read/Write events.
 * Returns: void.
 */
void Serial::Close()
{
	if (isOpened())
	{
		// Set timeout settings back to original.
		SetCommTimeouts(handler, &origTimeouts);
		// Close REad and Write Events to avoid handle leak.
		CloseHandle(osRead.hEvent);
		CloseHandle(osWrite.hEvent);
		// Close communication port.
		CloseHandle(handler);
		handler = INVALID_HANDLE_VALUE;
	}
}

/**
 * Read - Reads from a serial comm port into a buffer.
 * @success: Flag to be set as true when read operation is successful.
 * Returns: Returns a buffer containing the data read.
 */
char *Serial::Read(bool &success)
{
	success = false;
	if (!isOpened())
	{
		return 0;
	}

	DWORD dwRead;
	DWORD length = 11;
	BYTE *data = (BYTE *)(&rxdata);
	// Overlapped read operation
	if (!fWaitOnRead)
	{
		// Issue read operation.
		if (!ReadFile(handler, data, length, &dwRead, &osRead))
		{
			std::cout << "Here" << std::endl;
			if (GetLastError() != ERROR_IO_PENDING)
			{	/*Error*/
				// std::cout << "ERROR_IO_PENDING" << std::endl;
			}
			else
			{
				// std::cout << "Waiting" << std::endl;
				fWaitOnRead = TRUE; /*Waiting*/
			}
		}
		else
		{
			success = true;
		} // success
	}

	// Detect completion of an overlapped read operation
	DWORD dwRes;
	if (fWaitOnRead)
	{
		dwRes = WaitForSingleObject(osRead.hEvent, READ_TIMEOUT);
		switch (dwRes)
		{
		// Read completed.
		case WAIT_OBJECT_0:
			if (!GetOverlappedResult(handler, &osRead, &dwRead, FALSE))
			{ /*Error*/
			}
			else
			{
				if (dwRead == length)
					success = true;
				fWaitOnRead = FALSE;
				// Reset flag so that another opertion can be issued.
			} // Read completed successfully.
			break;

		case WAIT_TIMEOUT:
			// Operation isn't complete yet.
			break;

		default:
			// Error in the WaitForSingleObject;
			break;
		}
	}
	// std::cout << "Done" << rxdata << std::endl;
	return rxdata;
}


int Serial::readSerialPort(char *buffer, unsigned int buf_size)
{
	DWORD bytesRead;
	unsigned int toRead = 0;

	ClearCommError(handler, &errors, &status);

	if (this->status.cbInQue > 0)
	{
		if (status.cbInQue > buf_size)
		{
			toRead = buf_size;
		}
		else
			toRead = status.cbInQue;
	}

	if (ReadFile(handler, buffer, toRead, &bytesRead, NULL))
		return bytesRead;

	return 0;
}

// std::vector<char> Serial::ReadString(bool &success)
//{
// success = false;
// if (!isOpened())
//{
// return std::vector<char>(); // Return an empty vector if port is not open
//}
//
//	const int bufferSize = 1024;		  // Adjust buffer size as needed
// std::vector<char> buffer(bufferSize); // Buffer to store the read data
// DWORD bytesRead = 0;

// The creation of the overlapped read operation
// if (!fWaitOnRead)
//{
// Issue read operation
// if (!ReadFile(handler, buffer.data(), bufferSize, &bytesRead, &osRead))
//{
// if (GetLastError() != ERROR_IO_PENDING)
//{
// Error occurred
// return std::vector<char>();
//}
// else
//{
// fWaitOnRead = TRUE; // Waiting
//}
//}
// else
//{
// success = true; // Read completed successfully
// return std::vector<char>(buffer.begin(), buffer.begin() + bytesRead);
//}
//}

// Detection of the completion of an overlapped read operation
// DWORD dwRes;
// if (fWaitOnRead)
//{
//	dwRes = WaitForSingleObject(osRead.hEvent, READ_TIMEOUT);
//	switch (dwRes)
//	{
// Read completed
//	case WAIT_OBJECT_0:
//		if (!GetOverlappedResult(handler, &osRead, &bytesRead, FALSE))
//{
// Error occurred
// return std::vector<char>();
//}
// else
//{
// success = true; // Read completed successfully
// fWaitOnRead = FALSE;
// return std::vector<char>(buffer.begin(), buffer.begin() + bytesRead);
//}
// break;

// case WAIT_TIMEOUT:
//  Operation isn't complete yet
// break;

// default:
//  Error in the WaitForSingleObject
// break;
//}
//}

// return std::vector<char>(); // Return an empty vector if no data is read
//}

bool Serial::Write(char *buffer, int length)
{
	if (!isOpened())
		return (false);

	BOOL wSuccess = true;
	DWORD bytesWritten;

	length = std::clamp(length, 0, 1024);

	if (!WriteFile(handler, buffer, length, &bytesWritten, &osWrite))
	{
		// If WriteFile failed, check if write is pending or there is no delay.
		if (GetLastError() != ERROR_IO_PENDING)
			wSuccess = false;
		else
		{
			if (!GetOverlappedResult(handler, &osWrite, &bytesWritten, TRUE))
				wSuccess = false;
			else
				wSuccess = true;
		}
	}
	return wSuccess;
}

bool Serial::setRTS(bool value)
{
	if (isOpened())
	{
		if (value)
		{
			if (EscapeCommFunction(handler, SETRTS))
				return (true);
		}
		else
		{
			if (EscapeCommFunction(handler, CLRRTS))
				return (true);
		}
	}
	return (false);
}

bool Serial::setDTR(bool value)
{
	if (isOpened())
	{
		if (value)
		{
			if (EscapeCommFunction(handler, SETDTR))
				return (true);
		}
		else
		{
			if (EscapeCommFunction(handler, CLRDTR))
				return (true);
		}
	}
	return (false);
}

bool Serial::getCTS(bool &success)
{
	success = false;
	if (isOpened())
	{
		DWORD dwModemStatus;
		if (GetCommModemStatus(handler, &dwModemStatus))
		{
			success = true;
			return (MS_CTS_ON & dwModemStatus);
		}
	}
	return false;
}

bool Serial::getDSR(bool &success)
{
	success = false;
	if (isOpened())
	{
		DWORD dwModemStatus;
		if (GetCommModemStatus(handler, &dwModemStatus))
		{
			success = true;
			return (MS_DSR_ON & dwModemStatus);
		}
	}
	return (false);
}

bool Serial::getRI(bool &success)
{
	success = false;
	if (isOpened())
	{
		DWORD dwModemStatus;
		if (GetCommModemStatus(handler, &dwModemStatus))
		{
			success = true;
			return (MS_RING_ON & dwModemStatus);
		}
	}
	return (false);
}

bool Serial::getCD(bool &success)
{
	success = false;
	if (isOpened())
	{
		DWORD dwModemStatus;
		if (GetCommModemStatus(handler, &dwModemStatus))
		{
			success = true;
			return (MS_RLSD_ON & dwModemStatus);
		}
	}
	return (false);
}

bool SelectComPort() // added function to find the present serial
{
	char lpTargetPath[5000]; // buffer to store the path of the COMPORTS
	bool gotPort = false;	 // in case the port is not found

	for (int i = 0; i < 255; i++) // checking ports from COM0 to COM255
	{
		std::string str = "COM" + std::to_string(i); // converting to COM0, COM1, COM2
		DWORD test = QueryDosDevice(str.c_str(), lpTargetPath, 5000);

		// Test the return value and error if any
		if (test != 0) // QueryDosDevice returns zero if it didn't find an object
		{
			std::cout << str << ": " << lpTargetPath << std::endl;
			gotPort = true;
		}

		if (::GetLastError() == ERROR_INSUFFICIENT_BUFFER)
		{
		}
	}

	return gotPort;
}

#else // Functionality for Linux.

bool Serial::isOpened()
{
	return (serialFd != -1);
}

bool Serial::configureTermios()
{
	// Get port attributes.
	if (tcgetattr(serialFd, &tty) != 0)
	{
		std::cerr << "Error getting serial port attributes: "
				  << strerror(errno) << std::endl;
		return false;
	}

	// memset (&tty, 0, sizeof(tty));

	// Reconfigure some port settings.
	// Control Modes.
	// Parity.
	if (parity != 'N')
		tty.c_cflag |= PARENB;
	if (parity == 'O')
		tty.c_cflag |= PARODD;
	if (parity == 'N')
		tty.c_cflag &= PARENB;

	// Stop Bits.
	if (stopBits == 2)
		tty.c_cflag |= CSTOPB;
	else
		tty.c_cflag &= ~CSTOPB; // 1 stop bit.

	// Data Size.
	tty.c_cflag &= ~CSIZE; // Clear data size.
	if (dataSize == 5)
		tty.c_cflag |= CS5;
	else if (dataSize == 6)
		tty.c_cflag |= CS6;
	else if (dataSize == 7)
		tty.c_cflag |= CS7;
	else
		tty.c_cflag |= CS8; // 8 bits per byte.

	tty.c_cflag &= ~CRTSCTS;	   // Disable hardware flow control.
	tty.c_cflag |= CREAD | CLOCAL; // Turn on READ &
								   // ignore ctrl lines.

	// Local Modes.
	tty.c_lflag &= ~ICANON; // Disable canonical mode.
	tty.c_lflag &= ~ECHO;	// DIsable echo.
	tty.c_lflag &= ~ECHOE;	// Disable erasure.
	tty.c_lflag &= ~ECHONL; // DIsable new-line echo.
	tty.c_lflag &= ~ISIG;	// Disable interpretation of INTR, QUIT and SUSP

	// Input MOdes.
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control.
	// DIsable any special handling of received bytes.
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
	tty.c_oflag &= ~OPOST; // Prevent special intepretation of output bytes.
	tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/ Line feed.

	tty.c_cc[VTIME] = 1; // Timeout for non-blocking read.
	tty.c_cc[VMIN] = 0;	 // MInimum number of characters to read.
	// Set baud rate.
	if (!stdBaud)
		// Do Something if not standard baudRate.
		;
	cfsetispeed(&tty, baudRate);
	cfsetospeed(&tty, baudRate);

	// Save new configurations.
	if (tcsetattr(serialFd, TCSANOW, &tty) != 0)
	{
		std::cerr << "Error setting serial port attributes: "
				  << strerror(errno) << std::endl;
		return false;
	}

	return true;
}

int Serial::Open()
{
	serialFd = open(portName.c_str(), O_RDWR);
	if (serialFd == -1)
	{
		std::cerr << "Error opening serial port: "
				  << strerror(errno) << std::endl;
		return (-1);
	}
	// Sleep for a while as arduino uno resets when port is opened.
	sleep(3);
	if (configureTermios())
		return (0);
	else
		return (-1);
}

void Serial::Close()
{
	if (isOpened())
		close(serialFd);
	serialFd = -1;
}

void Serial::setBaudRate(long bRate)
{
	stdBaud = true;
	switch (bRate)
	{
	case 0:
		baudRate = B0;
		break;
	case 50:
		baudRate = B50;
		break;
	case 75:
		baudRate = B75;
		break;
	case 110:
		baudRate = B110;
		break;
	case 134:
		baudRate = B134;
		break;
	case 150:
		baudRate = B150;
		break;
	case 200:
		baudRate = B200;
		break;
	case 300:
		baudRate = B300;
		break;
	case 600:
		baudRate = B600;
		break;
	case 1200:
		baudRate = B1200;
		break;
	case 2400:
		baudRate = B2400;
		break;
	case 4800:
		baudRate = B4800;
		break;
	case 9600:
		baudRate = B9600;
		break;
	case 19200:
		baudRate = B19200;
		break;
	case 38400:
		baudRate = B38400;
		break;
	case 57600:
		baudRate = B57600;
		break;
	case 115200:
		baudRate = B115200;
		break;
	case 230400:
		baudRate = B230400;
		break;
	default:
		baudRate = bRate;
		stdBaud = false;
		break;
	}
}

std::string Serial::Read(unsigned int numChars, bool &rSuccess)
{
	char buffer[numChars];
	int bytesRead = read(serialFd, buffer, numChars);

	if (bytesRead == -1)
	{
		rSuccess = false; // Read operation failed
		return "";		  // Return empty string
	}
	rSuccess = true;					   // Read operation succeeded
	return std::string(buffer, bytesRead); // Convert char buffer to string
}

std::vector<char> Serial::Read(unsigned int numChars, bool &rsuccess, int x)
{
	rsuccess = false;

	std::vector<char> buffer(numChars);

	if (!isOpened())
		return (buffer);

	// Read data from serial port
	int bytesRead = read(serialFd, buffer.data(), numChars);

	if (bytesRead == -1)
	{
		// Failed to read from serial port
		return (buffer);
	}

	rsuccess = true;
	return (buffer);
}

bool Serial::Write(char *buffer, int length)
{
	if (!isOpened())
		return (false);

	length = std::clamp(length, 0, 1024);

	return (write(serialFd, buffer, length) == length);
}

bool Serial::setRTS(bool value)
{
	long RTS_flag = TIOCM_RTS;
	if (value)
	{ // Set RTS pin
		if (ioctl(serialFd, TIOCMBIS, &RTS_flag) == -1)
			return (false);
	}
	else
	{ // Clear RTS pin
		if (ioctl(serialFd, TIOCMBIC, &RTS_flag) == -1)
			return (false);
	}
	return (true);
}

bool Serial::setDTR(bool value)
{
	long DTR_flag = TIOCM_DTR;
	if (value)
	{ // Set DTR pin
		if (ioctl(serialFd, TIOCMBIS, &DTR_flag) == -1)
			return (false);
	}
	else
	{ // Clear DTR pin
		if (ioctl(serialFd, TIOCMBIC, &DTR_flag) == -1)
			return (false);
	}
	return (true);
}

bool Serial::getCTS(bool &success)
{
	success = true;

	long status;
	if (ioctl(serialFd, TIOCMGET, &status) == -1)
		success = false;
	return ((status & TIOCM_CTS) != 0);
}

bool Serial::getDSR(bool &success)
{
	success = true;

	long status;
	if (ioctl(serialFd, TIOCMGET, &status) == -1)
		success = false;
	return ((status & TIOCM_DSR) != 0);
}

bool Serial::getRI(bool &success)
{
	success = true;

	long status;
	if (ioctl(serialFd, TIOCMGET, &status) == -1)
		success = false;
	return ((status & TIOCM_RI) != 0);
}

bool Serial::getCD(bool &success)
{
	success = true;

	long status;
	if (ioctl(serialFd, TIOCMGET, &status) == -1)
		success = false;
	return ((status & TIOCM_CD) != 0);
}

using std::cout;
namespace fs = std::filesystem;

std::vector<std::string> getAvailablePorts()
{
	std::vector<std::string> port_names;

	fs::path p("/dev/serial/by-id");
	try
	{
		if (!exists(p))
		{
			throw std::runtime_error(p.generic_string() + " does not exist");
		}
		else
		{
			for (auto de : fs::directory_iterator(p))
			{
				if (is_symlink(de.symlink_status()))
				{
					fs::path symlink_points_at = read_symlink(de);
					fs::path canonical_path = fs::canonical(p / symlink_points_at);
					// cout << canonical_path.generic_string() << std::endl;
					port_names.push_back(canonical_path.generic_string());
				}
			}
		}
	}
	catch (const fs::filesystem_error &ex)
	{
		cout << ex.what() << '\n';
		throw ex;
	}
	std::sort(port_names.begin(), port_names.end());
	return port_names;
}

#endif
