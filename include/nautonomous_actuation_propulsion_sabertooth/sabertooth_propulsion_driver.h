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

        enum SabertoothCommand
        {
            Motor1Forward = 0,
            Motor1Backward = 1,

            Motor2Forward = 4,
            Motor2Backward = 5,

            MixedForward = 8,
            MixedBackward = 9,

            MixedRight = 10,
            MixedLeft = 11,
        };

        const uint8_t max_command_value_ = 127;

        float minimum_x_value_ = 0.01;
        float maximum_x_value_ = 1.0;

        float minimum_theta_value_ = 0.01;
        float maximum_theta_value_ = 1.57; //TODO is this the correct value?

        float minimum_motor_value_ = 0.01;
        float maximum_motor_value_ = 1.0;


        int sabertoothScale(double variable, double limit);

        void fillMixedModePacket(double value, double max_value, double min_value, int positive_command, int negative_command, uint8_t* packet);

        void fillRegularModePacket(double value, double max_value, double min_value, int positive_command, int negative_command, uint8_t* packet);

    public:
	 SabertoothMotorDriver* sabertooth_motor_driver_ = nullptr;

        SabertoothPropulsionDriver(ros::NodeHandle private_node_handle);
        ~SabertoothPropulsionDriver();

        void processTwist(const geometry_msgs::Twist::ConstPtr& twist_message, uint8_t* straight_packet, uint8_t* turn_packet);

        void processLeft(const std_msgs::Float32::ConstPtr& left_message, uint8_t* left_packet);

        void processRight(const std_msgs::Float32::ConstPtr& right_message, uint8_t* right_packet);

};

#endif // SABERTOOTHPROPULSIONDRIVER_HPP_
