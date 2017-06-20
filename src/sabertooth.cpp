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

double limit_variable(double variable, double limit){
	if(variable < 0.0){
		variable = std::max(variable, -limit);
	} else if(variable > 0.0){
		variable = std::min(variable, limit);
	}
	return variable;
}

void sabertooth_individual_propulsion_linear_model(uint8_t* left_motor_command, uint8_t* right_motor_command, const geometry_msgs::Twist::ConstPtr& twist) {
  const double max_component_value = 1.0;
	const double max_theta = 1.570; // 90 degrees
	const double medium_theta = 0.785; 
	const double max_magnitude = 1.0;
	
  const int max_propulsion_value = 127;

	double theta = twist->angular.z; 
	ROS_INFO("theta %f ", theta);
	theta = limit_variable(theta, max_theta);
	ROS_INFO("theta %f ", theta);
	
  double magnitude = twist->linear.x;
	magnitude = limit_variable(magnitude, max_magnitude);
	magnitude = std::max(sqrt(pow(cos(theta),2)+pow(sin(theta),2)), magnitude);

  // Fill in the sabertooth command.
  // address
  left_motor_command[0] = propulsion_address;
  right_motor_command[0] = propulsion_address;

	  // Compute the motor command values.
	int left_motor_value = max_propulsion_value * magnitude;
	int right_motor_value = max_propulsion_value * magnitude;
	
	if(theta < 0) {
		right_motor_value = max_propulsion_value * magnitude * (theta + medium_theta) / medium_theta;
	} else if (theta > 0) {
		left_motor_value = max_propulsion_value * magnitude * (medium_theta - theta) / medium_theta;
	}

  // command index for forward and backwards 
  int forward_command[2] = {0, 4}; // commands to drive motor {1, 2} forward.
  int backward_command[2] = {1, 5}; // commands to drive motor {1, 2} backward.

	int left_motor_command_value = (int) left_motor_value;
  if (left_motor_value > 0) {
    left_motor_command[1] = forward_command[left_motor_index]; // drive left motor forward.
  }
  else {
    left_motor_command[1] = backward_command[left_motor_index]; // drive left motor backward.
		// value is negative, switch it to positive;
		left_motor_command_value = -left_motor_command_value;
	}

	int right_motor_command_value = (int) right_motor_value;
  if (right_motor_value > 0) {
    right_motor_command[1] = forward_command[right_motor_index]; // drive right motor forward.
	}
  else {
    right_motor_command[1] = backward_command[right_motor_index]; // drive right motor backward.
		// value is negative, switch it to positive;
		right_motor_command_value = -right_motor_command_value;
  }

  // data/value
  left_motor_command[2] = left_motor_command_value;
  right_motor_command[2] = right_motor_command_value;
  ROS_ASSERT(left_motor_command[2] <= max_propulsion_value && left_motor_command[2] >= 0);
	ROS_ASSERT(right_motor_command[2] <= max_propulsion_value && right_motor_command[2] >= 0);

  // checksum
  left_motor_command[3] = (left_motor_command[0] + left_motor_command[1] + left_motor_command[2]) & 0b01111111;
  right_motor_command[3] = (right_motor_command[0] + right_motor_command[1] + right_motor_command[2]) & 0b01111111;

  // Publish motor inputs (for parameter identification)
	if(debug_motors){
		// x y z pos, theta,  scale x
		publish_arrow_marker(0, 0, theta+max_theta, magnitude, 0, g_twist_pub);
		publish_arrow_marker(-1.0, 0, max_theta, left_motor_value/(double)max_propulsion_value, 1, g_left_pub);
		publish_arrow_marker(1.0, 0, max_theta, right_motor_value/(double)max_propulsion_value, 2, g_right_pub);
	
	}

  ROS_INFO("Sabertooth left motor command %d %d %d %d", left_motor_command[0], left_motor_command[1], left_motor_command[2], left_motor_command[3]);
  ROS_INFO("Sabertooth right motor command %d %d %d %d", right_motor_command[0], right_motor_command[1], right_motor_command[2], right_motor_command[3]);
}

void publish_arrow_marker(double position_x, double position_y, double theta, double magnitude, int i, ros::Publisher g_pub){

	visualization_msgs::Marker marker;
	marker.header.frame_id = "motor_frame";
	marker.header.stamp = ros::Time::now();
	marker.ns = "marker_test_arrows";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = position_x;
	marker.pose.position.y = position_y;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = sin(theta/2);
	marker.pose.orientation.w = cos(theta/2);
	ROS_INFO("magnitude %f", magnitude);
	marker.scale.x = magnitude;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	if(i == 0){
		marker.color.r = 1.0;
	} else {
		marker.color.r = 0.0;
	}
	if(i == 1){
		marker.color.g = 1.0;
	} else {
		marker.color.g = 0.0;
	}
	if(i == 2){
		marker.color.b = 1.0;
	} else {
		marker.color.b = 0.0;
	}
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();
	marker.frame_locked = false;
	marker.text = "";
	marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
	marker.mesh_use_embedded_materials = 0;
	g_pub.publish(marker);

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