#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <cstring>
#include <sstream>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions

class SerialPort {
	private:
		char *port;
		int fd;
		struct termios tty_options, tty_options_old;
		bool connected;

		void connect(bool c);

	public:
		SerialPort();
		~SerialPort();
		void setFd(int fd);
		int getFd();
		void setPort(char *port);
		char* getPort();
		bool isConnected();		

		bool connectSerial();
		bool disconnectSerial();
		void configureSerial();
//		int sendSerialData(char* data, int size);
		void clearSerialData(int index=0);
		void updateSerial();


};
#endif // SERIAL_PORT_HPP
