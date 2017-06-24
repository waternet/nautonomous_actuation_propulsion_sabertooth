/*
 * command_actuation_node.hpp
 *
 *  Created on: Apr 16, 2016
 *      Author: zeeuwe01
 */

#ifndef COMMANDACTUATIONNODE_HPP_
#define COMMANDACTUATIONNODE_HPP_

#include <signal.h>

#include "ros/ros.h"

#include "send_actuation.hpp"
#include "sabertooth.hpp"
#include "watchdog.hpp"

#include "nautonomous_propulsion_msgs/IndividualMotorPropulsion.h"

void shutdownHandler(int sig);

nautonomous_propulsion_msgs::IndividualMotorPropulsion* individual_motor_propulsion_message = nullptr;

ros::Publisher pub_motor_inputs;

// Parameters
int driver_mode;
int left_motor_index;
int right_motor_index;

#endif /* COMMANDACTUATIONNODE_HPP_ */
