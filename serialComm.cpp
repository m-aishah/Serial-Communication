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

	std::vector<std::string> vec = getAvailablePorts();
	for (const auto& str : vec) {
        std::cout << str << std::endl;
    }
	std::cout << "Connection established!\n(ON/OFF/q)? ";
	std::cin >> msg;

	while (strcmp(msg, "q"))
	{
		if (!port.Write(msg, sizeof(msg)))
		{
			std::cerr << "Error writing to port." << std::endl;
			return 1;
		}
		sleep(1);

		char readBuf[256];
		bool success;
		strcpy(readBuf, (port.Read(sizeof(readBuf), success)).c_str());
		//int numBytes = port.readFromPort(readBuf, sizeof(readBuf));
		if (!readBuf)
		{
			std::cerr << "Error reading from port." << std::endl;
			return (1);
		}
		//readBuf[numBytes] = '\0';
		std::cout << "Read " << " bytes. Received message: " << readBuf
			  << std::endl;

		std::cout << "(ON/OFF/q)? ";
		std::cin >> msg;
	}

	std::cout << "Bye!" << std::endl;

	return 0;
}
