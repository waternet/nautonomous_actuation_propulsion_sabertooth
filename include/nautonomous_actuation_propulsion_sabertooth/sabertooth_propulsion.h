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
        ActuationWatchdog* actuation_watchdog;
        SabertoothPropulsionDriver* sabertooth_propulsion_driver;
        SabertoothSerial* sabertooth_serial;

        ros::Subscriber left_propulsion_subscriber;
        ros::Subscriber right_propulsion_subscriber;
        ros::Subscriber twist_propulsion_subscriber;

        void callback_propulsion_twist(const geometry_msgs::Twist::ConstPtr& twist_message);

        void callback_propulsion_left(const std_msgs::Float32::ConstPtr& left_message);
        void callback_propulsion_right(const std_msgs::Float32::ConstPtr& right_message);

    public:
        SabertoothPropulsion();
        ~SabertoothPropulsion();

        void checkWatchdog();
        
};

#endif // SABERTOOTHPROPULSION_H_