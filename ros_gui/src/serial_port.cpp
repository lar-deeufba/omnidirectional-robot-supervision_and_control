#include "../include/ros_gui/serial_port.hpp"

#define PORT_LEN 24

using namespace std;

SerialPort::SerialPort(){
    port = NULL;
    fd = -11;
    connect(false);
}

SerialPort::~SerialPort(){
    delete port;
}

//sets and gets methods
void SerialPort::setFd(int fd){
    this->fd = fd;
}

int SerialPort::getFd(){
    return fd;
}

void SerialPort::setPort(char *port){
    this->port = new char[PORT_LEN];

    strcpy(this->port, port);
}

char* SerialPort::getPort(){
    return port;
}

void SerialPort::connect(bool c){
    connected = c;
}

bool SerialPort::isConnected(){
    return connected;
}

//public functions
bool SerialPort::connectSerial(){
    fd = open(port, O_RDWR | O_NOCTTY);// | O_NONBLOCK | O_NDELAY);

    if(fd == -1) {
        cout << "Invalid port value. Couldn't open device\n";
        return false;
    }

    cout << "Connected to the device\n";
    connect(true);
    configureSerial();

    return true;
}

bool SerialPort::disconnectSerial(){
    if(fd == -1)
        return false;

    close(fd);
    //setFd(-1);
    connect(false);
    return true;
}

void SerialPort::updateSerial(){
    if(fd != -1){
        close(fd);
    }
    port = NULL;
    fd = -11;
    connect(false);
}

void SerialPort::configureSerial(){

    //Get the current options for the port...
    tcgetattr(fd, &tty_options);

    //Set the baud rates to 115200...
    cfsetispeed(&tty_options, B115200);
    cfsetospeed(&tty_options, B115200);

    /* Setting other Port Stuff */
    tty_options.c_cflag     &=  ~PARENB;        // Make 8n1
    tty_options.c_cflag     &=  ~CSTOPB;
    tty_options.c_cflag     &=  ~CSIZE;
    tty_options.c_cflag     |=  CS8;


    //Enable the receiver and set local mode...
    //tty_options.c_cflag |= (CLOCAL | CREAD);
    //tty_options.c_cflag |= (CLOCAL | CREAD | CRTSCTS);


    //tty_options.c_cflag     &=  ~CRTSCTS;       // no flow control

    //tty_options.c_iflag = IGNPAR;               //Ignore characters with parity errors.

    //tty_options.c_cc[VMIN]      =   48;                  // read doesn't block
    //tty_options.c_cc[VTIME]     =   0.1;                // 0.5 seconds read timeout

    //options.c_lflag &= ~(ICANON | ECHO | ISIG);
    //tty_options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    //tty_options.c_lflag = 0;


    tty_options.c_cc[VMIN] = 48;
    tty_options.c_cc[VTIME] = 0;

    tty_options.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                         ONOCR | OFILL | OLCUC | OPOST);

    tty_options.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR |
                        PARMRK | INPCK | ISTRIP | IXON) ;

    tty_options.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);


    //make it raw
    cfmakeraw(&tty_options);

    /* Flush Port, then applies attributes */
    tcflush( fd, TCIFLUSH );

    //Set the new options for the port...
    if (tcsetattr(fd, TCSANOW, &tty_options) != 0)
    {
        cout << "Error2 " << errno << " from tcsetattr" << endl;
    }
}


void SerialPort::clearSerialData(int index){
    int qs;
    switch(index){
    case 0:
          qs = TCIFLUSH;
          break;
    case 1:
          qs = TCOFLUSH;
          break;
    case 2:
          qs = TCIOFLUSH;
          break;
    default:
          qs = TCIFLUSH;
          break;
    };

    /* Flush Port, then applies attributes */
    tcflush( fd, qs );
    //std::cout << "index = " << index;
}
