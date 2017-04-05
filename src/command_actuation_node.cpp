#include "../include/nautonomous_actuation_synchronizer/command_actuation_node.hpp"

/**
 * \brief Shutdown handler, deinit serial, call ros::shutdown()
 * \param int sig
 * \return null 
 */

void shutdownHandler(int sig)
{
  // Deinit serial before shutdown down.
  actuation_deinit_serial();
  ROS_INFO("Shutting down deinitted serial");
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

/**
 * Main for command actuation node, subscribes to topics, call init serial
 */

int main(int argc, char **argv)
{

    ros::init(argc, argv, "command_actuation_node");
    ROS_INFO("Initializing command_actuation_node");
    ros::NodeHandle n;

    // Load parameters
    ros::NodeHandle nh_private("~");
    nh_private.param("driver_mode", driver_mode, sabertooth_mode::INDEPENDENT);
    nh_private.param("left_motor_index", left_motor_index, 0);
    nh_private.param("right_motor_index", right_motor_index, 1);
    ROS_WARN_STREAM("driver_mode: " << driver_mode << "  left_motor_index: " << left_motor_index << "  right_motor_index: " << right_motor_index); //TODO: remove if working...

    // Params
    ros::NodeHandle param("~");
    param.param("testing", testing_sabertooth, false);

    int propulsion_address_value = 0, conveyor_address_value = 0;
    param.param("propulsion_address", propulsion_address_value, 128);
    param.param("conveyor_address", conveyor_address_value, 132);
    propulsion_address = (uint8_t) propulsion_address_value;
    conveyor_address = (uint8_t) conveyor_address_value;
    ROS_INFO("Set propulsion_address to %u and conveyor_address to %u", propulsion_address, conveyor_address);

    // Subscribe to propulsion, conveyor and lighting topics.
    ros::Subscriber propulsionSub = n.subscribe("multiplexed_propulsion", 1000, actuation_send_propulsion_twist);
    ros::Subscriber conveyorSub = n.subscribe("multiplexed_conveyor", 1000, actuation_send_conveyor_twist);
    ros::Subscriber lightingSub = n.subscribe("multiplexed_lighting", 1000, actuation_send_lighting_bool);
    ros::Subscriber independentInputsSub = n.subscribe("independent_inputs", 1000, actuation_send_independent_inputs);

    pub_motor_inputs = n.advertise<nautonomous_msgs::IndependentInputs>("motor_inputs", 1, true);

    //Init the serial port for the motors
    //<!--TODO check if the serial connections was innited correctly.
    //actuation_init_serial();
    ROS_INFO("Subscribed to topics and serial initted");

    signal(SIGINT, shutdownHandler);

    ros::spin();

    return 0;
}
