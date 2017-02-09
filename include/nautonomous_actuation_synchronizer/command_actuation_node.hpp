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

void shutdownHandler(int sig);

#endif /* COMMANDACTUATIONNODE_HPP_ */
