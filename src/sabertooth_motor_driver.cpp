#include <nautonomous_actuation_propulsion_sabertooth/sabertooth_motor_driver.h>

SabertoothMotorDriver::SabertoothMotorDriver(uint8_t motor_address)
{
	address_ = motor_address;
}

/**
 *\brief Create test message for sabertooth
 *\param uint8_t address
 *\param uint8_t command
 *\param uint8_t value
 *\param uint8_t* sabertoothCommand
 *\return
 */
void SabertoothMotorDriver::fillPacket(uint8_t command, uint8_t value, uint8_t* packet)
{
	packet[0] = address_;
	packet[1] = command;
	packet[2] = value;

	setChecksum(packet);
}

void SabertoothMotorDriver::setChecksum(uint8_t* packet)
{
    packet[3] = (packet[0] + packet[1] + packet[2]) & 0b01111111; // 0b01111111 checksum constant to prevent overflow to the 8th bit.
}

/**
* Create two message for the timeout for the propulsion motor drivers.
* timeout in milliseconds (ms)
*/
void SabertoothMotorDriver::setSerialTimeout(uint16_t timeout, uint8_t* packet)
{
    // 14 is the command for serial timeout
	// 100 ms per value increase starting at 0, so 1000 ms timeout has a command of 10.
	fillPacket(14, (int) timeout / 100, packet);
}
