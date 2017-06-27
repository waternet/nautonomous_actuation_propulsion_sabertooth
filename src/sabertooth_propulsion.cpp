#include <nautonomous_actuation_propulsion_sabertooth/sabertooth_propulsion.h>

SabertoothPropulsion::SabertoothPropulsion()
{
	actuation_watchdog = new ActuationWatchdog();

	sabertooth_propulsion_driver = new SabertoothPropulsionDriver();

	sabertooth_serial = new SabertoothSerial();

	ros::NodeHandle node_handle;
	left_propulsion_subscriber = node_handle.subscribe("/actuation/propulsion/left", 10, &SabertoothPropulsion::callback_propulsion_left, this);
	right_propulsion_subscriber = node_handle.subscribe("/actuation/propulsion/right", 10, &SabertoothPropulsion::callback_propulsion_right, this);
	twist_propulsion_subscriber = node_handle.subscribe("/actuation/propulsion/twist", 10, &SabertoothPropulsion::callback_propulsion_twist, this);
}

SabertoothPropulsion::~SabertoothPropulsion()
{
	if(actuation_watchdog)
	{
		delete actuation_watchdog;
		actuation_watchdog = nullptr;
	}

	if(sabertooth_propulsion_driver)
	{
		delete sabertooth_propulsion_driver;
		sabertooth_propulsion_driver = nullptr;
	}

	if(sabertooth_serial)
	{
		delete sabertooth_serial;
		sabertooth_serial = nullptr;
	}
}

void SabertoothPropulsion::checkWatchdog()
{
    if(sabertooth_serial->isOpen())
	{
		std::string response;
        sabertooth_serial->readStatus(response); // only one byte

		actuation_watchdog->checkStatus(response);

        sabertooth_serial->flushInput();
    } 
	else
	{
        actuation_watchdog->setUnconnectedStatus(); 
    }
}


/**
 *\brief Send propulsion twist msg.
 *\params const geometry_msgs::Twist propulsion
 *\return
 */
void SabertoothPropulsion::callback_propulsion_twist(const geometry_msgs::Twist::ConstPtr& twist_message)
{
	if(sabertooth_serial->isOpen() && actuation_watchdog->isActive())
	{
		uint8_t straight_packet[4], turn_packet[4];

		sabertooth_propulsion_driver->processTwist(twist_message, &straight_packet[0], &turn_packet[0]);

		sabertooth_serial->writePacket(&straight_packet[0]);

		ros::Duration(0.01).sleep(); //TODO how fast can this be?

		sabertooth_serial->writePacket(&turn_packet[0]);
	}
}

void SabertoothPropulsion::callback_propulsion_left(const std_msgs::Float32::ConstPtr& left_message)
{
	if(sabertooth_serial->isOpen() && actuation_watchdog->isActive())
	{
		uint8_t left_packet[4];

		sabertooth_propulsion_driver->processLeft(left_message, &left_packet[0]);

		sabertooth_serial->writePacket(&left_packet[0]);
	}
}

void SabertoothPropulsion::callback_propulsion_right(const std_msgs::Float32::ConstPtr& right_message)
{
	if(sabertooth_serial->isOpen() && actuation_watchdog->isActive())
	{
		uint8_t right_packet[4];

		sabertooth_propulsion_driver->processRight(right_message, &right_packet[0]);

		sabertooth_serial->writePacket(&right_packet[0]);
	}
}

// /**
//  * Receive topic for independent inputs propulsion and create a new message based on the received topic message.
//  */
// void callback_propulsion_left_motor(const nautonomous_actuation_msgs::DifferentialMotor::ConstPtr& differential) {

// 	uint8_t left_motor_command[4], right_motor_command[4];	
// 	sabertooth_propulsion_driver->processDifferntial(differential, &left_motor_command[0], &right_motor_command[0]);

//   	if(sabertooth_serial.isOpen())
// 	{
//     	//Send to the first motor driver
//     	int bytes = actuation_serial->write(&left_motor_command[0], 4);

//     	ros::Duration(0.01).sleep();
//     	//Send to the second motor driver
//     	bytes += actuation_serial->write(&right_motor_command[0], 4);
//   	}
// }

// /**
//  *\brief Send propulsion twist msg.
//  *\params const geometry_msgs::Twist propulsion
//  *\return
//  */
// void actuation_independent_propulsion_twist(const geometry_msgs::Twist::ConstPtr& propulsion) {

// 	uint8_t leftCommand[4], rightCommand[4];

// 	sabertooth_individual_propulsion_linear_model(&leftCommand[0], &rightCommand[0], propulsion);

// 	if(serial_available){
// 		if(actuation_serial){
// 			//Check status watchdog
// 			if(status_msg.level == 0){
// 				//ROS_INFO("Actuation running");
				
// 				int bytes = actuation_serial->write(&leftCommand[0], 4);
// 				ros::Duration(0.01).sleep();
// 				bytes += actuation_serial->write(&rightCommand[0], 4);

// 			} else {
// 				//ROS_INFO("Actuation not running");
// 			}
			
// 		}
// 	}
// }
