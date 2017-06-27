#ifndef SABERTOOTHPROPULSION_H_
#define SABERTOOTHPROPULSION_H_

#include <stdint.h>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

#include "actuation_watchdog.h"
#include "sabertooth_propulsion_driver.h"
#include "sabertooth_serial.h"


class SabertoothPropulsion{
    private:
        ActuationWatchdog* actuation_watchdog_ = nullptr;
        SabertoothPropulsionDriver* sabertooth_propulsion_driver_ = nullptr;
        SabertoothSerial* sabertooth_serial_ = nullptr;

        ros::Subscriber left_propulsion_subscriber_;
        ros::Subscriber right_propulsion_subscriber_;
        ros::Subscriber twist_propulsion_subscriber_;

        void callbackPropulsionTwist(const geometry_msgs::Twist::ConstPtr& twist_message);

        void callbackPropulsionLeft(const std_msgs::Float32::ConstPtr& left_message);
        void callbackPropulsionRight(const std_msgs::Float32::ConstPtr& right_message);

    public:
        SabertoothPropulsion(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle);
        ~SabertoothPropulsion();

        void checkWatchdog();
        
};

#endif // SABERTOOTHPROPULSION_H_