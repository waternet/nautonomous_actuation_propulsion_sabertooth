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

#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "serial.hpp"
#include "sabertooth.hpp"
#include "ros/ros.h"

using namespace std;

serial::Serial* actuation_serial;

bool testing_sabertooth;

bool actuation_init_serial();
void actuation_deinit_serial();

size_t actuation_send_independent_inputs(double& left_motor_input, double& right_motor_input);

void actuation_send_propulsion_twist(const geometry_msgs::Twist::ConstPtr& propulsion);
void actuation_send_conveyor_twist(const geometry_msgs::Twist::ConstPtr& conveyor);
void actuation_send_lighting_bool(const std_msgs::Bool::ConstPtr& lighting);

#endif /* SENDACTUATION_HPP_ */
