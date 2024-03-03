#include "serial.h"

int main()
{
	Serial port;
	// Open port.
	if (port.Open() == -1)
	{
		std::cerr << "Failed to open port." << std::endl;
		return 1;
	}

	// std::vector<std::string> ports = port.listAvailablePorts();
	// for (int i = 0; i < 10; i++)
	// {
	//	std::cout << ports[i] << std::endl;
	// }

	char msg[255];

	// std::vector<std::string> vec = getAvailablePorts();
	SelectComPort();
	// for (const auto& str : vec) {
	//  std::cout << str << std::endl;
	//}
	std::cout << "Connection established!\n(ON/OFF/q)? ";
	std::cin >> msg;

	while (strcmp(msg, "q"))
	{
		if (!port.Write(msg, sizeof(msg)))
		{
			std::cerr << "Error writing to port." << std::endl;
			return 1;
		}
		Sleep(1);

		char readBuf[1024];
		bool success;
		int readBytes;
		Sleep(3000);
		strcpy(readBuf, port.Read(success));

		if (!success)
		{
			std::cerr << "Error reading from port." << std::endl;
			return (1);
		}

		std::cout << "Read "
				  << " bytes. Received message: " << readBuf
				  << std::endl;

		std::cout << "(ON/OFF/q)?  ";
		std::cin >> msg;
	}

	std::cout << "Bye!" << std::endl;

	return 0;
}
