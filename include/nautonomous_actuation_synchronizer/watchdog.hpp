/*
 * command_actuation_node.hpp
 *
 *  Created on: Apr 16, 2016
 *      Author: zeeuwe01
 */

#ifndef WATCHDOG_HPP_
#define WATCHDOG_HPP_

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "send_actuation.hpp"

#include "diagnostic_msgs/DiagnosticStatus.h"

ros::Publisher watchdog_publisher;

diagnostic_msgs::DiagnosticStatus status_msg;
diagnostic_msgs::DiagnosticStatus temp_status_msg;

void check_status();
void run_watchdog();

#endif /* WATCHDOG_HPP_ */