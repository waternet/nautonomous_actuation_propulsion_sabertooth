/*
 * SendActuation.cpp
 *
 *  Created on: Apr 16, 2016
 *      Author: zeeuwe01
 */

#include "../include/nautonomous_actuation_synchronizer/send_actuation.hpp"

/**
 *\brief  Initialized the serial connection, set the used port and bautrate.
 *\params
 *\return
 */
bool actuation_init_serial() {
	ROS_INFO("Opening Sabertooth drive, testing %d", test_sabertooth);

	if(!test_sabertooth){
		ROS_INFO("Opening Sabertooth driver");
		//Open serial port sabertooth at 9600 baud with 250 ms timeout.
		actuation_serial = new serial::Serial(string("/dev/nautonomous/actuation"),115200,
				serial::Timeout::simpleTimeout(250));
		ROS_INFO("Serial open: %d", actuation_serial->isOpen());

		if(actuation_serial->isOpen()){
			ros::Duration(1).sleep();
			
			//Prepare the serial timeout for both motor drivers and send them.
			uint8_t serial_timeout_propulsion[4], serial_timeout_conveyor[4];
			sabertooth_advanced_serial_timeout(&serial_timeout_propulsion[0], &serial_timeout_conveyor[0]);

			//Write timeout to propulsion motor driver
			int bytes = actuation_serial->write(&serial_timeout_propulsion[0], 4);

			ros::Duration(0.01).sleep();

			//Write timeout to conveyor belt motor driver
			bytes += actuation_serial->write(&serial_timeout_conveyor[0], 4);
			return bytes;
		} else {
			//Exit false since we could not open the actuation_serial connection.
			return false;
		}
	} else { 

	 	ROS_INFO("Opening Sabertooth driver");
		//Open serial port sabertooth at 9600 baud with 250 ms timeout.

		actuation_serial = new serial::Serial(string("/dev/nautonomous/actuation"),115200, serial::Timeout::simpleTimeout(250));
		ROS_INFO("Serial open: %d", actuation_serial->isOpen());

		return actuation_serial->isOpen();
	}
}



/**
 *\brief  Deinitialize and removes the serial connection.
 *\params
 *\return
 */
void actuation_deinit_serial() {
	if(actuation_serial){
		actuation_serial->close();
		delete actuation_serial;
		actuation_serial = nullptr;
	}
}

/**
 *\brief  Deprecated, Send debug twist.
 *\params const geometry_msgs::Twist propulsion
 *\return
 */
void actuation_send_debug_twist(const geometry_msgs::Twist::ConstPtr& propulsion) {

}

/**
 *\brief Send propulsion twist msg.
 *\params const geometry_msgs::Twist propulsion
 *\return
 */
void actuation_send_propulsion_twist(const geometry_msgs::Twist::ConstPtr& propulsion) {

	uint8_t straightCommand[4], turnCommand[4];

	sabertooth_advanced_process_propulsion_twist(&straightCommand[0], &turnCommand[0], propulsion);



	if(actuation_serial && !test_sabertooth){
		//Check status watchdog
		if(status_msg.level == 0){
			//ROS_INFO("Actuation running");
			
			int bytes = actuation_serial->write(&straightCommand[0], 4);
			ros::Duration(0.01).sleep();
			bytes += actuation_serial->write(&turnCommand[0], 4);

		} else {
			//ROS_INFO("Actuation not running");
		}
		
	}
}

/**
 *\brief  Send conveyor twist.
 *\params const geometry_msgs::Twist conveyor
 *\return
 */
void actuation_send_conveyor_twist(const geometry_msgs::Twist::ConstPtr& conveyor){
	ROS_INFO("Conveyor message");
	//do nothing
}

/**
 *\brief  Send lighting boolean.
 *\params const std_msgs::Bool lighting
 *\return
 */
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

    	ros::Duration(0.01).sleep();

		  ROS_INFO("Sending twist message");
    	//Send to the second motor driver
    	bytes += actuation_serial->write(&right_motor_command[0], 4);
	#else
    	ROS_INFO("Simulated");
	#endif
  	} else {
  	}
}

/**
 *\brief Send propulsion twist msg.
 *\params const geometry_msgs::Twist propulsion
 *\return
 */
void actuation_independent_propulsion_twist(const geometry_msgs::Twist::ConstPtr& propulsion) {

	uint8_t leftCommand[4], rightCommand[4];

	sabertooth_individual_propulsion_linear_model(&leftCommand[0], &rightCommand[0], propulsion);

	if(serial_available){
		if(actuation_serial){
			//Check status watchdog
			if(status_msg.level == 0){
				//ROS_INFO("Actuation running");
				
				int bytes = actuation_serial->write(&leftCommand[0], 4);
				ros::Duration(0.01).sleep();
				bytes += actuation_serial->write(&rightCommand[0], 4);

			} else {
				//ROS_INFO("Actuation not running");
			}
			
		}
	}
}