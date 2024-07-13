# SerialCommCPP 📡

*A versatile C++ library for seamless serial communication across Windows and Linux!*

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [API Reference](#api-reference)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## Introduction

Welcome to **SerialCommCPP**! This library is your go-to solution for handling serial communication in your C++ projects. Whether you're interfacing with microcontrollers, sensors, or any device that speaks over serial, we've got you covered. Compatible with both Windows and Linux, SerialCommCPP makes cross-platform serial communication a breeze! 🌍💻

## Features 🌟

- **Cross-platform compatibility**: Works on both Windows and Linux.
- **Easy to use**: Simple API to open, read, and write to serial ports.
- **Configurable**: Set baud rate, data size, parity, and stop bits to match your device's requirements.
- **RTS, DTR control**: Control and query modem control lines.
- **Port detection**: List available serial ports (Linux) or COM ports (Windows).

## Installation 🚀

Clone the repository to get started:

```bash
git clone https://github.com/m-aishah/SerialCommCPP.git
cd SerialCommCPP
```

## Usage 🛠️

To use the SerialCommCPP library, simply compile and run the provided `main.cpp` file:

```bash
g++ -o serial_comm main.cpp serial.cpp
./serial_comm
```

This will execute the main program that utilizes the SerialCommCPP library.

## API Reference 📚

### Constructor
```cpp
Serial();
Serial(std::string device, long bRate, long dSize, char pType, float sBits);
```
- **device**: Serial port name (e.g., `/dev/ttyUSB0` or `COM7`).
- **bRate**: Baud rate (e.g., `9600`).
- **dSize**: Data size (e.g., `8`).
- **pType**: Parity type (e.g., `'N'` for none).
- **sBits**: Stop bits (e.g., `1`).

### Open and Close
```cpp
int Open();
void Close();
```

### Read and Write
```cpp
char *Read(bool &rSuccess);
std::vector<char> Read(unsigned int numChars, bool &rsuccess, int x);
std::string Read(unsigned int numChars, bool &rSuccess);
bool Write(char *buffer, int length);
```

### Configuration
```cpp
void setPortName(std::string device);
void setBaudRate(long bRate);
void setDataSize(long dSize);
void setParity(char pType);
void setStopBits(float sBits);
std::string getPortName();
long getBaudRate();
long getDataSize();
char getParity();
float getStopBits();
```

### Modem Control
```cpp
bool setRTS(bool value);
bool setDTR(bool value);
bool getCTS(bool &success);
bool getDSR(bool &success);
bool getRI(bool &success);
bool getCD(bool &success);
```

### Helpers
```cpp
bool isOpened();
void Delay(unsigned long ms);
```

### Port Detection
```cpp
std::vector<std::string> getAvailablePorts(); // Linux
bool SelectComPort(); // Windows
```

## Contributing 💡

Contributions are welcome! If you have ideas for improvements or find bugs, feel free to fork the repository and open a pull request. Let's make SerialCommCPP even better together! 🛠️

1. Fork the repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Commit your changes (`git commit -m 'Add some feature'`).
4. Push to the branch (`git push origin feature-branch`).
5. Open a pull request.

## License 📜

This project is licensed under the MIT License.

## Contact 📧

For questions or support, reach out to Aishah Ayomide Mabayoje at [maishah2540@gmail.com](mailto:maishah2540@gmail.com).

---

Happy coding with SerialCommCPP! 🎉