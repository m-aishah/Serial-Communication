#ifndef SERIAL_H
#define SERIAL_H
#include <cstdlib>
#include <cstdio>
#include <string>
#include <cstring>

// If in Windows environment define WINDOWS else define LINUX
#if defined(__WINDOWS__) || defined(_WIN64) || defined(__WIN32__) || defined(_WIN32) || defined(WIN32) || defined(__CYGWIN__) || defined(__TOS_WIN__)
	#define WINDOWS
#elif defined(__unix__) || defined(__unix) || defined(unix)
	#define LINUX
#endif

// Include windows.h if in Windows.
#ifdef WINDOWS
	#include <windows.h>
	#define READ_TIMEOUT 10
	#define ARDUINO_WAIT_TIME 60
#else
	#include <termios.h>
	#include <linux/serial.h>
	#include <sys/ioctl.h>
	#include <unistd.h>
	#include <fctnl.h>
#endif

class Serial {
	private:
		char *rxString;
		std::string portName;
		long baudRate;
		long dataSize;
		char parity;
		float stopBits;
		bool stdBaud;

	#ifdef WINDOWS
		HANDLE handler;
		OVERLAPPED osRead;
		OVERLAPPED osWrite;
		// BOOL fWaitOnRead;
		COMMTIMEOUTS origTimeouts;
	#else
		long serialFd;
	#endif
	
	public:
		Serial();
		Serial(std::string device, long bRate, long dSize, char pType, float sBits);
		~Serial();
		int open(); // Return 0 on success, otherwise return -1.
		void close();
		std::string Read(bool& success, long maxLength);
		bool Write(char *buffer, long length);
		
		// Set/Get config settings.
		bool setRTS(bool value);
		bool setDTR(bool valus);
		bool getCTS(bool &success);
		bool getDSR(bool &success);
		bool getRI(bool &success);
		bool getCD(bool &success);

		void setPortName(std::string device);
		void setBaudRate(long bRate);
		void setDataSize(long dSize);
		void setParity(char pType);
		void setStopBits(float sBits);

		std::string getPortName();
		long getBaudRate();
		long getDataSize();
		char getParity();
		float getStopBIts();

		// Helpers.
		bool isOpened();
		static void Delay(unsigned long ms);
};

void Serial::Delay(unsigned long ms)
{
#ifdef WINDOWS
	Sleep(ms);
#else
	usleep(ms * 1000);
#endif
}

#endif // SERIAL_H
