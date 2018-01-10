#include "../include/ros_gui/serialCommunication.hpp"

SerialCommunication::SerialCommunication(){
}

SerialCommunication::~SerialCommunication(){
}

 void SerialCommunication::serialSend(string address, int command, char *parameter, int sizeOfParameter, int fd) {

    memset(buffer,0,sizeof buffer);

    int messageSize;

   buffer[0] = char(0x7E);
   buffer[1] = char(0x00);
   buffer[3] = char(0x10);
   buffer[4] = char(0x00);

   int addressSize = address.length();

   if (addressSize < 16) {
       int n;
       for(n=0; n < (16 - addressSize); n++ ) {
           address = '0' + address;
       }
   }

   char *p = &buffer[5];

    messageSize = strtobyte(address,&buffer[5]); // Converter

   buffer[13] = char(0xFF);
   buffer[14] = char(0xFE);
   buffer[15] = char(0x00);

   buffer[16] = char(0x00);
   buffer[17] = char(command);

   if (sizeOfParameter > 0) {

     messageSize = sizeOfParameter;

     int i;
     for(i=0;i<sizeOfParameter;i++) {

         buffer[18+i] = parameter[i];

     }

     buffer[18 + messageSize] = char(checkSum(&buffer[3],messageSize + 15));

   }
   else {
       buffer[18] = char(checkSum(&buffer[3],15));
       messageSize = 0;
   }

  buffer[2] = char(messageSize+15);

  write(fd, buffer, sizeof buffer);

}

int SerialCommunication::checkSum(char *buffer, int size) {

    int n;
    int soma=0;

    for(n=0; n< size; n++) {
        soma += buffer[n];
    }

     return (0xFF - soma);

}


int SerialCommunication::strtobyte(string src, char* dest) {

    char tmp[3];

    unsigned int a;
    int n=0, j=0;
    int len = src.length();

    tmp[2] = '\0';

    for(n=0; n<= len-1; n+=2) {

        tmp[0] = src[n];
        if(n+1 < len)
            tmp[1] = src[n+1];
        else
            tmp[1] = '\0';

        sscanf(tmp, "%x", &a);
        dest[j++] = (unsigned char) a;

    }

    return j;

}
