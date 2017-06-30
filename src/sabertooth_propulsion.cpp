#include <nautonomous_actuation_propulsion_sabertooth/sabertooth_propulsion.h>

SabertoothPropulsion::SabertoothPropulsion(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
{
	actuation_watchdog_ = new ActuationWatchdog(node_handle);

	sabertooth_propulsion_driver_ = new SabertoothPropulsionDriver(private_node_handle);

	sabertooth_serial_ = new SabertoothSerial(private_node_handle);

	private_node_handle.param("propulsion_differential_mode", differential_mode_, false);

	if (differential_mode_)
	{
		left_propulsion_subscriber_ = node_handle.subscribe("left_motor_topic", 10, &SabertoothPropulsion::callbackPropulsionLeft, this);
		right_propulsion_subscriber_ = node_handle.subscribe("right_motor_topic", 10, &SabertoothPropulsion::callbackPropulsionRight, this);
	}
	else
	{
		twist_propulsion_subscriber_ = node_handle.subscribe("cmd_vel_topic", 10, &SabertoothPropulsion::callbackPropulsionTwist, this);
	}
}

SabertoothPropulsion::~SabertoothPropulsion()
{
	if(actuation_watchdog_)
	{
		delete actuation_watchdog_;
		actuation_watchdog_ = nullptr;
	}

	if(sabertooth_propulsion_driver_)
	{
		delete sabertooth_propulsion_driver_;
		sabertooth_propulsion_driver_ = nullptr;
	}

	if(sabertooth_serial_)
	{
		delete sabertooth_serial_;
		sabertooth_serial_ = nullptr;
	}
}

void SabertoothPropulsion::checkWatchdog()
{
    if(sabertooth_serial_->isOpen())
	{
		std::string response;
        sabertooth_serial_->readStatus(response); // only one byte

		actuation_watchdog_->checkStatus(response);

        sabertooth_serial_->flushInput();
	} 
	else
	{
        actuation_watchdog_->setNotConnectedStatus(); 
    }
}

/**
 *\brief Send propulsion twist msg.
 *\params const geometry_msgs::Twist propulsion
 *\return
 */
void SabertoothPropulsion::callbackPropulsionTwist(const geometry_msgs::Twist::ConstPtr& twist_message)
{
	if(sabertooth_serial_->isOpen() && actuation_watchdog_->isActive())
	{
		uint8_t straight_packet[4], turn_packet[4];

		sabertooth_propulsion_driver_->processTwist(twist_message, &straight_packet[0], &turn_packet[0]);

		sabertooth_serial_->writePacket(&straight_packet[0]);

		ros::Duration(0.01).sleep(); //TODO how fast can this be?

		sabertooth_serial_->writePacket(&turn_packet[0]);
	}
}

void SabertoothPropulsion::callbackPropulsionLeft(const std_msgs::Float32::ConstPtr& left_message)
{
	if(sabertooth_serial_->isOpen() && actuation_watchdog_->isActive())
	{
		uint8_t left_packet[4];

		sabertooth_propulsion_driver_->processLeft(left_message, &left_packet[0]);

		sabertooth_serial_->writePacket(&left_packet[0]);
	}
}

void SabertoothPropulsion::callbackPropulsionRight(const std_msgs::Float32::ConstPtr& right_message)
{
	if(sabertooth_serial_->isOpen() && actuation_watchdog_->isActive())
	{
		uint8_t right_packet[4];

		sabertooth_propulsion_driver_->processRight(right_message, &right_packet[0]);

		sabertooth_serial_->writePacket(&right_packet[0]);
	}
}
