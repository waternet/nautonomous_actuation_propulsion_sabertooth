/*
 * send_actuation.hpp
 *
 *  Created on: Apr 16, 2016
 *      Author: zeeuwe01
 */

#ifndef SENDACTUATION_HPP_
#define SENDACTUATION_HPP_

#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <nautonomous_propulsion_msgs/IndividualMotorPropulsion.h>

#include <sabertooth.hpp>
#include <serial.hpp>
#include <std_msgs/Bool.h>

#include <watchdog.hpp>

using namespace std;

serial::Serial* actuation_serial;

bool serial_available;

string actuation_serial_port;

bool actuation_init_serial();
void actuation_deinit_serial();

void actuation_send_propulsion_twist(const geometry_msgs::Twist::ConstPtr& propulsion);
void actuation_send_conveyor_twist(const geometry_msgs::Twist::ConstPtr& conveyor);
void actuation_send_lighting_bool(const std_msgs::Bool::ConstPtr& lighting);
void actuation_send_individual_motor_propulsion(const nautonomous_propulsion_msgs::IndividualMotorPropulsion::ConstPtr& msg);

void actuation_independent_propulsion_twist(const geometry_msgs::Twist::ConstPtr& propulsion);

#endif /* SENDACTUATION_HPP_ */
