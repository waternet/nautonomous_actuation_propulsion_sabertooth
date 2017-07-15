/**
    Sabertooth Propulsion
    sabertooth_propulsion.cpp
    Purpose: Delegating module that uses the serial, sabertooth and watchdog modules.

    @author Daan Zeeuwe
    @version 1.0 8/7/17 
*/

#include <nautonomous_actuation_propulsion_sabertooth/sabertooth_propulsion.h>

SabertoothPropulsion::SabertoothPropulsion(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
{
	//Create related modules
	actuation_watchdog_ = new ActuationWatchdog(node_handle);

	sabertooth_motor_driver_ = new SabertoothMotorDriver();

	sabertooth_serial_ = new SabertoothSerial(private_node_handle);

	//Get the params
	private_node_handle.param("propulsion_differential_mode_param", differential_mode_, false);

	private_node_handle.param("debug", debug_, false);

	
	private_node_handle.param("minimum_forward_value", minimum_forward_value_, 0.01);
    private_node_handle.param("maximum_forward_value", maximum_forward_value_, 1.0);

    private_node_handle.param("minimum_turning_value", minimum_turning_value_, 0.01);
    private_node_handle.param("maximum_turning_value", maximum_turning_value_, 0.5);

    private_node_handle.param("minimum_motor_value", minimum_motor_value_, 0.01);
    private_node_handle.param("maximum_motor_value", maximum_motor_value_, 1.0);
	
    // Motor address
    int motor_address = 128; // param does not allow uint8_t
    private_node_handle.param("propulsion_address", motor_address, 128); //128 default propulsion address

	//Create a sabertooth packet
	sabertooth_packet_.setAddress((uint8_t) motor_address);

	//Check watchdog
	checkWatchdog();

	//Set Timeout of motor
	int serial_timeout;
	private_node_handle.param("serial_timeout_ms", serial_timeout, 1000);
	sabertooth_motor_driver_->setSerialTimeout(&sabertooth_packet_, serial_timeout);
	queuePacket();

	//Get publishers
	left_feedback_publisher_ = node_handle.advertise<std_msgs::Int16>("feedback/left_feedback_topic", 1, true);
	right_feedback_publisher_ = node_handle.advertise<std_msgs::Int16>("feedback/right_feedback_topic", 1, true);

	//Get subscribers
	if (differential_mode_)
	{
		left_propulsion_subscriber_ = node_handle.subscribe("motor/left_motor_topic", 10, &SabertoothPropulsion::callbackPropulsionLeft, this);
		right_propulsion_subscriber_ = node_handle.subscribe("motor/right_motor_topic", 10, &SabertoothPropulsion::callbackPropulsionRight, this);
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

	if(sabertooth_motor_driver_)
	{
		delete sabertooth_motor_driver_;
		sabertooth_motor_driver_ = nullptr;
	}

	if(sabertooth_serial_)
	{
		delete sabertooth_serial_;
		sabertooth_serial_ = nullptr;
	}
}

void SabertoothPropulsion::checkWatchdog()
{
	std::string response = sabertooth_serial_->readStatus(); // only one byte
	actuation_watchdog_->checkStatus(response);

	sabertooth_serial_->flushInput();
}

/**
 *\brief Send propulsion twist msg.
 *\params const geometry_msgs::Twist propulsion
 *\return
 */
void SabertoothPropulsion::callbackPropulsionTwist(const geometry_msgs::Twist::ConstPtr& twist_message)
{
	sabertooth_motor_driver_->processMotorValue(&sabertooth_packet_, twist_message->linear.x, maximum_forward_value_, minimum_forward_value_, 
		SabertoothPacket::SabertoothCommand::MixedForward, SabertoothPacket::SabertoothCommand::MixedBackward);

	queuePacket();

	sabertooth_motor_driver_->processMotorValue(&sabertooth_packet_, twist_message->angular.z, maximum_turning_value_, minimum_turning_value_, 
		SabertoothPacket::SabertoothCommand::MixedLeft, SabertoothPacket::SabertoothCommand::MixedRight);

	queuePacket();
}	

void SabertoothPropulsion::callbackPropulsionLeft(const std_msgs::Float32::ConstPtr& left_message)
{	
	sabertooth_motor_driver_->processMotorValue(&sabertooth_packet_, left_message->data, maximum_motor_value_, minimum_motor_value_, 
		SabertoothPacket::SabertoothCommand::Motor1Forward, SabertoothPacket::SabertoothCommand::Motor1Backward);

	queuePacket();

	left_feedback_publisher_.publish(createFeedbackMessage());
}

void SabertoothPropulsion::callbackPropulsionRight(const std_msgs::Float32::ConstPtr& right_message)
{
	sabertooth_motor_driver_->processMotorValue(&sabertooth_packet_, right_message->data, maximum_motor_value_, minimum_motor_value_, 
		SabertoothPacket::SabertoothCommand::Motor2Forward, SabertoothPacket::SabertoothCommand::Motor2Backward);

	queuePacket();
	
	right_feedback_publisher_.publish(createFeedbackMessage());
}

void SabertoothPropulsion::queuePacket()
{
	if(debug_)
	{
		ROS_INFO("Packet: %i %i %i %i", sabertooth_packet_.getAddress(), sabertooth_packet_.getCommand(), sabertooth_packet_.getValue(), sabertooth_packet_.getChecksum());
	}

	if(actuation_watchdog_->isActive())
	{
		sabertooth_serial_->writePacket(sabertooth_packet_);
		ros::Duration(0.025).sleep(); //TODO how fast can this be?
	}
}

std_msgs::Int16 SabertoothPropulsion::createFeedbackMessage()
{
	std_msgs::Int16 feedback;

	feedback.data = (int) sabertooth_packet_.getValue();

	// If the command is a backwards command, then we have to negate the value.
	uint8_t command = sabertooth_packet_.getCommand();
	if(command == SabertoothPacket::SabertoothCommand::Motor1Backward || 
		command == SabertoothPacket::SabertoothCommand::Motor2Backward)
	{
		feedback.data = -feedback.data;
	}

	return feedback;
}