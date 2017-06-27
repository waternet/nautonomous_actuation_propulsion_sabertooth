#ifndef SABERTOOTHMOTORDRIVER_H_
#define SABERTOOTHMOTORDRIVER_H_

#include <stdint.h>

class SabertoothMotorDriver
{
    private:
         void setChecksum(uint8_t* packet);

         uint8_t address;

    public:
        SabertoothMotorDriver(uint8_t motor_address);

        void fillPacket(uint8_t command, uint8_t value, uint8_t* packet);
        void setSerialTimeout(uint16_t timeout, uint8_t* packet);

};

#endif // SABERTOOTHMOTORDRIVER_H_
