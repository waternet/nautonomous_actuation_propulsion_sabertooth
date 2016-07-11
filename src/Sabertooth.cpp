#include "../include/nautonomous_actuation_synchronizer/Sabertooth.hpp"
//
int propulsion_address_int = 128;
char propulsion_address = propulsion_address_int;
int conveyor_address_int = 132;
char conveyor_address = conveyor_address_int;

void sabertooth_advanced_process_propulsion_twist(uint8_t* straightCommand, uint8_t* turnCommand, geometry_msgs::Twist* twist){
	straightCommand[0] = propulsion_address;

	float minimumValue = 0.01;
	float maximumValue = 1.0;

	if(twist->linear.x > maximumValue){
		twist->linear.x = maximumValue;
	} else if(twist->linear.x < -maximumValue){
		twist->linear.x = -maximumValue;
	}

	if(twist->linear.x > minimumValue && twist->linear.x <= maximumValue)
	{
		straightCommand[1] = 8;
		int commandValue =(int)(127.0*(twist->linear.x / maximumValue));
		straightCommand[2] = commandValue;
		straightCommand[3] = (straightCommand[0]+straightCommand[1]+straightCommand[2]) & 0b01111111;
	}
	else if(twist->linear.x < -minimumValue && twist->linear.x >= -maximumValue)
	{
		straightCommand[1] = 9;
		int commandValue =(int)(127.0*(twist->linear.x / -maximumValue));
		straightCommand[2] = commandValue;
		straightCommand[3] = (straightCommand[0]+straightCommand[1]+straightCommand[2]) & 0b01111111;
	} else {
		straightCommand[1] = 9;
		int commandValue = 0;
		straightCommand[2] = commandValue;
		straightCommand[3] = (straightCommand[0]+straightCommand[1]+straightCommand[2]) & 0b01111111;
	}

	turnCommand[0] = propulsion_address;

	minimumValue = 0.01;
	maximumValue = 1.0;

	if(twist->angular.z > maximumValue){
		twist->angular.z = maximumValue;
	} else if(twist->angular.z < -maximumValue){
		twist->angular.z = -maximumValue;
	}

	if(twist->angular.z > minimumValue && twist->angular.z <= maximumValue)
	{
		turnCommand[1] = 11; //left
		int commandValue =(int)(127.0*(twist->angular.z / maximumValue));
		turnCommand[2] = commandValue;
		turnCommand[3] = (turnCommand[0]+turnCommand[1]+turnCommand[2]) & 0b01111111;
	}
	else if(twist->angular.z < -minimumValue && twist->angular.z >= -maximumValue)
	{
		turnCommand[1] = 10; //right
		int commandValue =(int)(127.0*(twist->angular.z / -maximumValue));
		turnCommand[2] = commandValue;
		turnCommand[3] = (turnCommand[0]+turnCommand[1]+turnCommand[2]) & 0b01111111;
	} else {
		turnCommand[1] = 10;
		int commandValue = 0;
		turnCommand[2] = commandValue;
		turnCommand[3] = (turnCommand[0]+turnCommand[1]+turnCommand[2]) & 0b01111111;
	}

	//ROS_INFO("Sabertooth straight command %d %d %d %d", straightCommand[0],straightCommand[1],straightCommand[2],straightCommand[3]);
	//ROS_INFO("Sabertooth turn command %d %d %d %d", turnCommand[0],turnCommand[1],turnCommand[2],turnCommand[3]);
}

void sabertooth_advanced_process_conveyor_twist(uint8_t* straightCommand, uint8_t* turnCommand, geometry_msgs::Twist* twist){
	straightCommand[0] = conveyor_address;

	float minimumValue = 0.01;
	float maximumValue = 1.0;

	if(twist->linear.x > maximumValue){
		twist->linear.x = maximumValue;
	} else if(twist->linear.x < -maximumValue){
		twist->linear.x = -maximumValue;
	}

	if(twist->linear.x > minimumValue && twist->linear.x <= maximumValue)
	{
		straightCommand[1] = 8;
		int commandValue =(int)(127.0*(twist->linear.x / maximumValue));
		straightCommand[2] = commandValue;
		straightCommand[3] = (straightCommand[0]+8+commandValue) & 0b01111111;
	}
	else if(twist->linear.x < -minimumValue && twist->linear.x >= -maximumValue)
	{
		straightCommand[1] = 9;
		int commandValue =(int)(127.0*(twist->linear.x / -maximumValue));
		straightCommand[2] = commandValue;
		straightCommand[3] = (straightCommand[0]+9+commandValue) & 0b01111111;
	} else {
		straightCommand[1] = 9;
		int commandValue = 0;
		straightCommand[2] = commandValue;
		straightCommand[3] = (straightCommand[0]+9+commandValue) & 0b01111111;
	}

	turnCommand[0] = conveyor_address;

	minimumValue = 0.01;
	maximumValue = 1.0;

	if(twist->angular.z > maximumValue){
		twist->angular.z = maximumValue;
	} else if(twist->angular.z < -maximumValue){
		twist->angular.z = -maximumValue;
	}

	if(twist->angular.z > minimumValue && twist->angular.z <= maximumValue)
	{
		turnCommand[1] = 11;
		int commandValue =(int)(127.0*(twist->angular.z / maximumValue));
		turnCommand[2] = commandValue;
		turnCommand[3] = (straightCommand[0]+11+commandValue) & 0b01111111;
	}
	else if(twist->angular.z < -minimumValue && twist->angular.z >= -maximumValue)
	{
		turnCommand[1] = 10;
		int commandValue =(int)(127.0*(twist->angular.z / -maximumValue));
		turnCommand[2] = commandValue;
		turnCommand[3] = (straightCommand[0]+10+commandValue) & 0b01111111;
	} else {
		turnCommand[1] = 10;
		int commandValue = 0;
		turnCommand[2] = commandValue;
		turnCommand[3] = (straightCommand[0]+10+commandValue) & 0b01111111;
	}

	//ROS_INFO("Sabertooth straight command %d %d %d %d", straightCommand[0],straightCommand[1],straightCommand[2],straightCommand[3]);
	//ROS_INFO("Sabertooth turn command %d %d %d %d", turnCommand[0],turnCommand[1],turnCommand[2],turnCommand[3]);
}

void sabertooth_advanced_serial_timeout(uint8_t* driver1, uint8_t* driver2){
	driver1[0] = propulsion_address;
	driver1[1] = 14;
	driver1[2] = 10;
	driver1[3] = (driver1[0]+driver1[1]+driver1[2]) & 0b01111111;
	
	driver2[0] = conveyor_address;
	driver2[1] = 14;
	driver2[2] = 10;
	driver2[3] = (driver2[0]+driver2[1]+driver2[2]) & 0b01111111;

	//ROS_INFO("Sabertooth driver1 command %d %d %d %d", driver1[0],driver1[1],driver1[2],driver1[3]);
	//ROS_INFO("Sabertooth driver2 command %d %d %d %d", driver2[0],driver2[1],driver2[2],driver2[3]);
}

