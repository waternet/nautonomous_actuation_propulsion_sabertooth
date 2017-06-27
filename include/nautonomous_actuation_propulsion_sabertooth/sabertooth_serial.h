
#ifndef SABERTOOTHSERIAL_H_
#define SABERTOOTHSERIAL_H_

#include <iostream>

#include <ros/ros.h>

#include "serial.h"

class SabertoothSerial
{
    private:
        serial::Serial* propulsion_serial_;

        std::string serial_port_;
        bool serial_available_;
        int serial_baud_; 
        int serial_timeout_;

        bool initSerial();
        bool initialize();
        
    public:
        SabertoothSerial(ros::NodeHandle private_node_handle);
        ~SabertoothSerial();

        bool isOpen();

        void readStatus(std::string response);
        void writePacket(uint8_t* packet);

        void flushInput();

};

#endif /* SABERTOOTHSERIAL_H_ */
