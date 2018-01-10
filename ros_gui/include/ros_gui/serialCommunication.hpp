/**
 * @file /include/ros_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef SERIAL_COMMUNICATION_HPP
#define SERIAL_COMMUNICATION_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string.h>
#include <termios.h>    // POSIX terminal control definitions
#include "serial_port.hpp"

using namespace std;

#define BUFFER_SIZE 255

/*****************************************************************************
** Class
*****************************************************************************/

class SerialCommunication {

private:
    char buffer[BUFFER_SIZE];

public:
    SerialCommunication();
    virtual ~SerialCommunication();

    void serialSend(string address, int command, char* parameter, int sizeOfParameter, int fd);
    static int checkSum(char *buffer, int size);
    int strtobyte(string src, char* dest);
	
};


#endif /* ros_gui_SERIAL_COMMUNICATION_HPP_ */
