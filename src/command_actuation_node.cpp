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

    // Testing and debugging
    ros::NodeHandle param("~");
    param.param("serial_available", serial_available, true);
    param.param("serial_port", actuation_serial_port, std::string("/dev/nautonomous/actuation"));

    param.param("test_motors", test_motors, false);
    param.param("debug_motors", debug_motors, false);
    ROS_INFO("Testing motors: %d", test_motors);
    ROS_INFO("Debugging motors: %d", debug_motors);

    // Motor addresses
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

    ros::Subscriber independentInputsSub = n.subscribe("motor_mode/independent_inputs", 1000, actuation_send_independent_inputs);
    ros::Subscriber independentInputsTwistSub = n.subscribe("motor_mode/twist/independent_inputs", 1000, actuation_independent_propulsion_twist);
    
    pub_motor_inputs = n.advertise<nautonomous_msgs::IndependentInputs>("motor_inputs", 1, true);

    g_twist_pub = n.advertise<visualization_msgs::Marker> ("visualization_marker_twist", 0);
    g_left_pub = n.advertise<visualization_msgs::Marker> ("visualization_marker_left", 0);
    g_right_pub = n.advertise<visualization_msgs::Marker> ("visualization_marker_right", 0);
    
    g_left_forward_pub = n.advertise<visualization_msgs::Marker> ("visualization_marker_left_forward", 0);
    g_right_forward_pub = n.advertise<visualization_msgs::Marker> ("visualization_marker_right_forward", 0);
    g_left_backward_pub = n.advertise<visualization_msgs::Marker> ("visualization_marker_left_backward", 0);
    g_right_backward_pub = n.advertise<visualization_msgs::Marker> ("visualization_marker_right_backward", 0);
    
    ROS_INFO("Subscribed to topics for multiplexer");

    //Publisher
    watchdog_publisher = n.advertise<diagnostic_msgs::DiagnosticStatus>("watchdog",
			1000);

    //Init the serial port for the motors
    if(serial_available){
        if(actuation_init_serial()){
          ROS_INFO("Actuation initted successfully");
          run_watchdog();
        } else{
          ROS_WARN("Could not init Actuation");
          // Error occured while opening the serial port.
          return 1;
        }
    } else {
      ROS_INFO("Serial is not available.");
    }

    signal(SIGINT, shutdownHandler);

    ros::spin();

    return 0;
}
