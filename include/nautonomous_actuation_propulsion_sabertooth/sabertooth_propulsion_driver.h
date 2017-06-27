#ifndef SABERTOOTHPROPULSIONDRIVER_HPP_
#define SABERTOOTHPROPULSIONDRIVER_HPP_

#include <stdio.h>                  // Need to be able to use standard output "putchar" command
#include <stdint.h>                 // Include your standard types definition file to accomodate the C99 uint_x types
#include <algorithm>                // std::max and std::min

#include <ros/ros.h>

#include <std_msgs/Float32.h>

#include <geometry_msgs/Twist.h>

#include "sabertooth_motor_driver.h"

class SabertoothPropulsionDriver
{
    private: 
        SabertoothMotorDriver* sabertooth_motor_driver_ = nullptr;

    public:
        SabertoothPropulsionDriver(ros::NodeHandle private_node_handle);
        ~SabertoothPropulsionDriver();

        void processTwist(const geometry_msgs::Twist::ConstPtr& twist_message, uint8_t* straight_command, uint8_t* turn_command);
        
        void processLeft(const std_msgs::Float32::ConstPtr& left_message, uint8_t* left_command);
        void processRight(const std_msgs::Float32::ConstPtr& right_message, uint8_t* right_command);

};

#endif // SABERTOOTHPROPULSIONDRIVER_HPP_
