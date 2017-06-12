#include "../include/nautonomous_actuation_synchronizer/sabertooth.hpp"

/**
 *\brief Create test message for sabertooth
 *\param uint8_t address
 *\param uint8_t command
 *\param uint8_t value
 *\param uint8_t* sabertoothCommand
 *\return
 */
void sabertooth_test_message(uint8_t address, uint8_t command, uint8_t value, uint8_t* sabertoothCommand){
	sabertoothCommand[0] = address;
	sabertoothCommand[1] = command;
	sabertoothCommand[2] = value;
	sabertoothCommand[3] = (address+command+value)&0b01111111;
}

/**
 *\brief Propulsion Create an array of straight command and turn command, using the twist message to translate from a twist message to a Sabertooth command array.
 *\param uint8_t* straightCommand
 *\param uint8_t* turnCommand
 *\param const geometry_msgs::Twist::ConstPtr& twist
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

	/*
		OPTION 1: only steering, no forward movement. Double the steering value
	*/
	//Spliting moving forward and steering. If angular.z is above a certain value, only steering and no straight
	if(twist->angular.z > 0.02 || twist->angular.z < -0.02){
		straightCommand[1] = 0;
		straightCommand[2] = 0;

		//Steering value x2 to make it stronger -> max of 127
		turnCommand[2] = turnCommand[2] * 2;
		if(turnCommand[2] > 127){
			turnCommand[2] = 127;
		}
	}

	/*
		OPTION 2: Both steering and moving forward, double the steering value


	turnCommand[2] = turnCommand[2] * 2;
	if(turnCommand[2] > 127){
		turnCommand[2] = 127;
	}
	*/

	turnCommand[3] = (turnCommand[0]+turnCommand[1]+turnCommand[2]) & 0b01111111;
	straightCommand[3] = (straightCommand[0]+straightCommand[1]+straightCommand[2]) & 0b01111111;

	ROS_INFO("Sabertooth straight command %d %d %d %d", straightCommand[0],straightCommand[1],straightCommand[2],straightCommand[3]);
	ROS_INFO("Sabertooth turn command %d %d %d %d", turnCommand[0],turnCommand[1],turnCommand[2],turnCommand[3]);

}

void sabertooth_advanced_process_propulsion_independent_inputs(uint8_t* left_motor_command, uint8_t* right_motor_command, const nautonomous_msgs::IndependentInputs::ConstPtr& msg) {
  double max_input = 1.0;

	float left_motor_input_temp = msg->left_motor_input;
	float right_motor_input_temp = msg->right_motor_input;

  // Make sure the requested inputs do not exceed the maximum allowed input (saturation).
  if (left_motor_input_temp > max_input) {
    left_motor_input_temp = max_input;
  } else if (left_motor_input_temp < -max_input) {
    left_motor_input_temp = -max_input;
  }
  if (right_motor_input_temp > max_input) {
    right_motor_input_temp = max_input;
  } else if (right_motor_input_temp < -max_input) {
    right_motor_input_temp = -max_input;
  }

  // Compute the motor command values.
	int left_motor_command_value = (int) fabs(127.0 * left_motor_input_temp / max_input) + 0.5; // +0.5 for rounding. //TODO: correct??
	int right_motor_command_value = (int) fabs(127.0 * right_motor_input_temp / max_input) + 0.5;

  // Fill in the sabertooth command.
  // address
  left_motor_command[0] = propulsion_address;
  right_motor_command[0] = propulsion_address;
  // command
  int forward_command[2] = {0, 4}; // commands to drive motor {1, 2} forward.
  int backward_command[2] = {1, 5}; // commands to drive motor {1, 2} backward.
  if (left_motor_input_temp > 0) {
    left_motor_command[1] = forward_command[left_motor_index]; // drive left motor forward.
  }
  else {
    left_motor_command[1] = backward_command[left_motor_index]; // drive left motor backward.
  }
  if (right_motor_input_temp > 0) {
    right_motor_command[1] = forward_command[right_motor_index]; // drive right motor forward.
  }
  else {
    right_motor_command[1] = backward_command[right_motor_index]; // drive right motor backward.
  }
  // data/value
  left_motor_command[2] = left_motor_command_value;
  right_motor_command[2] = right_motor_command_value;
  ROS_ASSERT(left_motor_command[2] < 128 && right_motor_command[2] < 128);
  // checksum
  left_motor_command[3] = (left_motor_command[0] + left_motor_command[1] + left_motor_command[2]) & 0b01111111;
  right_motor_command[3] = (right_motor_command[0] + right_motor_command[1] + right_motor_command[2]) & 0b01111111;

  // Publish motor inputs (for parameter identification)
  nautonomous_msgs::IndependentInputs motor_inputs;
  motor_inputs.left_motor_input = left_motor_input_temp;
  motor_inputs.right_motor_input = right_motor_input_temp;
  pub_motor_inputs.publish(motor_inputs);

  ROS_INFO("Sabertooth left motor command %d %d %d %d", left_motor_command[0], left_motor_command[1], left_motor_command[2], left_motor_command[3]);
  ROS_INFO("Sabertooth right motor command %d %d %d %d", right_motor_command[0], right_motor_command[1], right_motor_command[2], right_motor_command[3]);
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