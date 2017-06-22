/**********************************************************************************************
 *
 * Sabertooth_2x25_Driver.h
 * Version 2.0.2
 * 07 July, 2013
 *
 * Brad Hedges
 * H & H Enterprises
 * Stratford, Oklahoma
 *
 **********************************************************************************************
 **********************************************************************************************
 *
 * This is a multi-purpose device driver to operate the Dimension Engineering Sabertooth series
 * of H-Bridge motor controllers. These can be used to control permanent magnet DC motors
 * between 6 & 30 volts, up to 25 amps continuous and 50 amps surge.
 *
 * This driver works in the "Packetized Serial" mode, which must be set with the DIP
 * switches on the controller hardware. The driver defaults to operating only one
 * Sabertooth, using the lowest physical address of 128. The address is configurable in
 * the driver by assigning the address value to the constant "MOTOR_DRIVER_ADDRESS_x".
 *
 * The driver is written in the C language for Microchip PIC microcontrollers using CCS's
 * PIC-C Compiler, and was developed using version 4.120 of that compiler. I have also used it
 * with 32-bit PICs, using Microchip's C32 compiler.
 *
 * It should be easily convertible to other microcontrollers and compilers with minimal effort. I have
 * annotated the three lines which would have to be modified for a different compiler or
 * processor architecture.
 *
 * Dimension Engineering's web site is at:      www.dimensionengineering.com
 * CCS's web site is:                           www.ccsinfo.com
 * Microchip's web site:                        www.microchip.com
 *
 *********************************************************************************************/

#ifndef SABERTOOTH_HPP_
#define SABERTOOTH_HPP_

#include <stdio.h>                  // Need to be able to use standard output "putchar" command
#include <stdint.h>                 // Include your standard types definition file to accomodate the C99 uint_x types
#include <algorithm>                // std::max and std::min
#include <nautonomous_msgs/IndependentInputs.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

uint8_t propulsion_address;
uint8_t conveyor_address;

bool test_sabertooth;
bool test_motors;
bool debug_motors;

ros::Publisher g_twist_pub;
ros::Publisher g_left_pub;
ros::Publisher g_right_pub;

namespace sabertooth_mode {
static constexpr int INDEPENDENT = 0;
static constexpr int MIXED = 1;
}

void sabertooth_test_message(uint8_t address, uint8_t command, uint8_t value, uint8_t* sabertoothCommand);
void sabertooth_advanced_process_propulsion_twist(uint8_t* straightCommand, uint8_t* turnCommand, const geometry_msgs::Twist::ConstPtr& twist);
void sabertooth_advanced_process_conveyor_twist(uint8_t* motor1, uint8_t* motor2, const geometry_msgs::Twist::ConstPtr& twist);
void sabertooth_advanced_serial_timeout(uint8_t* driver1, uint8_t* driver2);
void sabertooth_advanced_process_propulsion_independent_inputs(uint8_t* left_motor_command, uint8_t* right_motor_command, const nautonomous_msgs::IndependentInputs::ConstPtr& msg);

void sabertooth_individual_propulsion_linear_model(uint8_t* left_motor_command, uint8_t* right_motor_command, const geometry_msgs::Twist::ConstPtr& twist);

double limit_variable(double variable, double limit);
void publish_arrow_marker(double position_x, double position_y, double theta, double magnitude, int i, ros::Publisher g_pub);

extern ros::Publisher pub_motor_inputs;
// Parameters
extern int driver_mode;
extern int left_motor_index;
extern int right_motor_index;

#endif
