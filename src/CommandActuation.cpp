#include "../include/nautonomous_actuation_synchronizer/CommandActuation.hpp"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "command_actuation");

  ros::NodeHandle n;
	// Subscribe to multiplexed propulsion, conveyor and lighting topics.
  ros::Subscriber propulsionSub = n.subscribe("multiplexed_propulsion", 1000, propulsionCallback);
  ros::Subscriber conveyorSub = n.subscribe("multiplexed_conveyor", 1000, conveyorCallback);
  ros::Subscriber lightingSub = n.subscribe("multiplexed_lighting", 1000, lightingCallback);
  ROS_INFO("Subscribed");
  //Init the serial port for the motors
  actuation_init_serial();
  ROS_INFO("Serial initted");

  ros::Rate r(25);
  while(ros::ok()){
	//Test if the messages are available, if so send the message and delete them.
	  if(propulsion_message){
		  actuation_send_twist(propulsion_message, true);
		  delete propulsion_message;
		  propulsion_message = nullptr;
	  }
	  if(conveyor_message){
		  actuation_send_twist(conveyor_message, false);
		  delete conveyor_message;
		  conveyor_message = nullptr;
	  }

	  r.sleep();
	  ros::spinOnce();
  }

  actuation_deinit_serial();

  return 0;
}
