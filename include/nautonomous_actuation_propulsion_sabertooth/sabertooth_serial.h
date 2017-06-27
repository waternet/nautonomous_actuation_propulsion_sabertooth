
#ifndef SABERTOOTHSERIAL_H_
#define SABERTOOTHSERIAL_H_

#include <iostream>

#include <ros/ros.h>

#include "serial.h"

class SabertoothSerial
{
    private:
        serial::Serial* propulsion_serial;

        std::string serial_port;
        bool serial_available;
        int serial_baud; 
        int serial_timeout;

        bool initSerial();
        bool initialize();
        
    public:
        SabertoothSerial();
        ~SabertoothSerial();

        bool isOpen();

        void readStatus(std::string response);
        void writePacket(uint8_t* packet);

        void flushInput();

};

#endif /* SABERTOOTHSERIAL_H_ */
