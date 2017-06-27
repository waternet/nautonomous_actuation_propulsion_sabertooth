
#include <nautonomous_actuation_propulsion_sabertooth/sabertooth_serial.h>

SabertoothSerial::SabertoothSerial()
{
    ros::NodeHandle node_handle("~");
    node_handle.param("serial_port", serial_port, std::string("/dev/nautonomous/actuation"));
    node_handle.param("serial_baud", serial_baud, 115200);
    node_handle.param("serial_timeout", serial_timeout, 250);
    node_handle.param("serial_available", serial_available, true);

    initialize();
}

SabertoothSerial::~SabertoothSerial()
{
    if(propulsion_serial)
	{
		propulsion_serial->close();
		delete propulsion_serial;
		propulsion_serial = nullptr;
	}
}

/**
 *\brief  Initialized the serial connection, set the used port and bautrate.
 *\params void
 *\return if the actuation serial is open or not
 */
bool SabertoothSerial::initSerial() 
{   
		propulsion_serial = new serial::Serial(serial_port, serial_baud,
		    serial::Timeout::simpleTimeout(serial_timeout));

		return isOpen();
}

/**
 * Init the serial port for the motors
 * Returns true if the serial connection was opened.
 */
bool SabertoothSerial::initialize()
{
    if(serial_available)
    {
        if(initSerial())
        {
            return 1;
        } 
        else
        {
            ROS_ERROR("Could not init Actuation");
        }
    } 
    else 
    {
        ROS_WARN("Serial is not available.");
    }

	return 0;
}

bool SabertoothSerial::isOpen() 
{
    if(propulsion_serial){
        return propulsion_serial->isOpen();
    }
	return false;
}

void SabertoothSerial::readStatus(std::string response)
{
    if(propulsion_serial){
        propulsion_serial->read(response, 1); //status mesage only has 1 byte
    }
}

void SabertoothSerial::writePacket(uint8_t* packet)
{
    if(propulsion_serial)
    {
        propulsion_serial->write(&packet[0], 4); // Packet has 4 arguments.
    }
}

void SabertoothSerial::flushInput()
{
    if(propulsion_serial){
        propulsion_serial->flushInput();
    }
}