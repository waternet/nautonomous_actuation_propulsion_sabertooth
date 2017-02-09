#include "../include/nautonomous_actuation_synchronizer/sabertooth.hpp"
//

void sabertooth_test_message(uint8_t address, uint8_t command, uint8_t value, uint8_t* sabertoothCommand){
	sabertoothCommand[0] = address;
	sabertoothCommand[1] = command;
	sabertoothCommand[2] = value;
	sabertoothCommand[3] = (address+command+value)&0b01111111;
}

/*
* Create an array of straight command and turn command, using the twist message to translate from a twist message to a Sabertooth command array.
*/
void sabertooth_advanced_process_propulsion_twist(uint8_t* straightCommand, uint8_t* turnCommand, const geometry_msgs::Twist::ConstPtr& twist){
	
	float minimumValue = 0.01;
	float maximumValue = 1.0;
	
	//Translate twist message to sabertooth propulsion command
	straightCommand[0] = propulsion_address;
	if(twist->linear.x > minimumValue)
	{
		straightCommand[1] = 8; //forward
		straightCommand[2] = (int)(127.0 * (std::min((float)twist->linear.x, maximumValue) / maximumValue));
	}
	else if(twist->linear.x < -minimumValue)
	{
		straightCommand[1] = 9; //backward
		straightCommand[2] = (int)(127.0 * (std::max((float)twist->linear.x, -maximumValue) / -maximumValue));
	}
	else 
	{ //does not have a linear speed above the minimum value, then send an speed of 0.
		straightCommand[1] = 9;
		straightCommand[2] = 0;
	}
	straightCommand[3] = (straightCommand[0]+straightCommand[1]+straightCommand[2]) & 0b01111111;


	minimumValue = 0.01;
	maximumValue = 0.5;

	//Translate twist message to sabertooth rotational command
	turnCommand[0] = propulsion_address;
	if(twist->angular.z > minimumValue)
	{
		turnCommand[1] = 11; //left
		turnCommand[2] = (int)(127.0 * (std::min((float)twist->angular.z, maximumValue) / maximumValue));
	}
	else if(twist->angular.z < -minimumValue)
	{
		turnCommand[1] = 10; //right
		turnCommand[2] = (int)(127.0*(std::max((float)twist->angular.z, -maximumValue) / -maximumValue));
	} 
	else 
	{ //does not have a rotational speed above the minimum value, then send an speed of 0.
		turnCommand[1] = 10;
		turnCommand[2] = 0;
	}
	turnCommand[3] = (turnCommand[0]+turnCommand[1]+turnCommand[2]) & 0b01111111;
}

/**
* Create two message for the timeout for both motor drivers.
*/
void sabertooth_advanced_serial_timeout(uint8_t* driver1, uint8_t* driver2){
	driver1[0] = propulsion_address;
	driver1[1] = 14;
	driver1[2] = 10;
	driver1[3] = (driver1[0]+driver1[1]+driver1[2]) & 0b01111111;
	
	driver2[0] = conveyor_address;
	driver2[1] = 14;
	driver2[2] = 10;
	driver2[3] = (driver2[0]+driver2[1]+driver2[2]) & 0b01111111;
}

