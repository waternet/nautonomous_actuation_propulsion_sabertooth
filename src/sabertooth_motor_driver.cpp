/**
    Sabertooth Motor Driver
    sabertooth_motor_driver.cpp
    Purpose: Transform ROS messages to Sabertooth packets content.

    @author Daan Zeeuwe
    @version 1.0 8/7/17 
*/

#include <nautonomous_actuation_propulsion_sabertooth/sabertooth_motor_driver.h>

SabertoothMotorDriver::SabertoothMotorDriver()
{

}

// Scale the value within the boundaries (0, max_command_value_).
int SabertoothMotorDriver::sabertoothScale(double value, double max_value, double min_value)
{   
    int motor_value = 0;

    if(value > min_value)
    {
        motor_value = max_command_value_ * (std::min(value, max_value) / max_value);
    } 
    else if(variable < -min_value)
    {
        motor_value = max_command_value_ * (std::max(value, -max_value) / -max_value); 
    } 
    
    return motor_value;
}

void SabertoothMotorDriver::processMotorValue(SabertoothPacket* packet, double motor_value, double max_value, double min_value, int positive_command, int negative_command)
{
    uint8_t command = 0;
    uint8_t value = 0;

    // Check if the value is positive and accordingly choose the correct command.
    command = (motor_value >= 0.0) ? (positive_command) : (negative_command);

    // Check if the value is larger than the min_value in the positive case and smaller in the negative case.
    value = sabertoothScale(motor_value, max_value, min_value);

    packet->fillPacket(command, value);
}

/**
* Create two message for the timeout for the Motor motor drivers.
* timeout in milliseconds (ms)
* @param uint16_t timeout_ms - 100 ms per value increase starting at 0, so 1000 ms timeout has a command of 10.
* @param uint8_t* packet
*/
void SabertoothMotorDriver::setSerialTimeout(SabertoothPacket* packet, uint16_t timeout_ms)
{
	packet->fillPacket(SabertoothPacket::SabertoothCommand::SerialTimeout, (int) timeout_ms / 100);
}
