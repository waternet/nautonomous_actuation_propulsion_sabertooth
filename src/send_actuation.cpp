/*
 * SendActuation.cpp
 *
 *  Created on: Apr 16, 2016
 *      Author: zeeuwe01
 */

#include "../include/nautonomous_actuation_synchronizer/send_actuation.hpp"

/**
*  Initialized the serial connection, set the used port and bautrate. 
**/

bool actuation_init_serial() {
	ROS_INFO("Opening Sabertooth drive, testing %d", testing_sabertooth);
	testing_sabertooth = false;

	if(!testing_sabertooth){
		ROS_INFO("Opening Sabertooth driver");
		//Open serial port sabertooth at 9600 baud with 250 ms timeout.
		actuation_serial = new serial::Serial(string("/dev/ttyACM0"),115200,
				serial::Timeout::simpleTimeout(250));
		ROS_INFO("Serial open: %d", actuation_serial->isOpen());

		//   (Deprecated, done by actuation platform)
		//ros::Duration(1).sleep();
		
		//Prepare the serial timeout for both motor drivers and send them.
		//uint8_t serial_timeout_propulsion[4], serial_timeout_conveyor[4];
		//sabertooth_advanced_serial_timeout(&serial_timeout_propulsion[0], &serial_timeout_conveyor[0]);

		//Write timeout to propulsion motor driver
		//int bytes = actuation_serial->write(&serial_timeout_propulsion[0], 4);

		//ros::Duration(0.01).sleep();

		//Write timeout to conveyor belt motor driver
		//bytes += actuation_serial->write(&serial_timeout_conveyor[0], 4);
		return 1;
	} else {
		return 0;
	}
}

void actuation_deinit_serial() {
	if(actuation_serial){
		actuation_serial->close();
		delete actuation_serial;
		actuation_serial = nullptr;
	}
}

void actuation_send_debug_twist(const geometry_msgs::Twist::ConstPtr& propulsion) {

}

void actuation_send_propulsion_twist(const geometry_msgs::Twist::ConstPtr& propulsion) {

	uint8_t straightCommand[4], turnCommand[4];

	sabertooth_advanced_process_propulsion_twist(&straightCommand[0], &turnCommand[0], propulsion);



	if(actuation_serial && !testing_sabertooth){
		int bytes = actuation_serial->write(&straightCommand[0], 4);

		ros::Duration(0.01).sleep();

		bytes += actuation_serial->write(&turnCommand[0], 4);
	}
}

void actuation_send_conveyor_twist(const geometry_msgs::Twist::ConstPtr& conveyor){
	//do nothing
}

void actuation_send_lighting_bool(const std_msgs::Bool::ConstPtr& lighting){
	// do nothing
}


/**
 * Receive topic for independent inputs propulsion and create a new message based on the received topic message.
 */
void actuation_send_independent_inputs(const nautonomous_msgs::IndependentInputs::ConstPtr& msg) {

	ROS_INFO("Indepentent callback");

	uint8_t left_motor_command[4], right_motor_command[4];	
	sabertooth_advanced_process_propulsion_independent_inputs(&left_motor_command[0], &right_motor_command[0], msg);

	//TODO: what is all this exactly??...
  	if(actuation_serial){
	#if not defined(SABERTOOTH_TEST)
    	//Send to the first motor driver
    	int bytes = actuation_serial->write(&left_motor_command[0], 4);

    	ros::Rate r(100);
    	r.sleep(); //wait 0.01
		  ROS_INFO("Sending twist message");
    	//Send to the second motor driver
    	bytes += actuation_serial->write(&right_motor_command[0], 4);
	#else
    	ROS_INFO("Simulated");
	#endif
  	} else {
  	}
}

