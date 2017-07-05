/*
 * command_actuation_node.hpp
 *
 *  Created on: Apr 16, 2016
 *      Author: zeeuwe01
 */

#ifndef SABERTOOTHPROPULSIONNODE_H_
#define SABERTOOTHPROPULSIONNODE_H_

#include <signal.h>

#include <ros/ros.h>

#include "sabertooth_propulsion.h"

void shutdownHandler(int sig);

SabertoothPropulsion* sabertooth_propulsion;

#endif /* SABERTOOTHPROPULSIONNODE_H_ */
