#include <nautonomous_actuation_propulsion_sabertooth/sabertooth_propulsion_driver.h>

SabertoothPropulsionDriver::SabertoothPropulsionDriver(ros::NodeHandle private_node_handle)
{
    // Motor address
    int address = 128; // param does not allow uint8_t
    private_node_handle.param("propulsion_address", address, 128); //128 default propulsion address
    //TODO exception when the address is not in range from the manual.

    sabertooth_motor_driver_ = new SabertoothMotorDriver((uint8_t) address);
}

SabertoothPropulsionDriver::~SabertoothPropulsionDriver()
{
    if(sabertooth_motor_driver_)
    {
        delete sabertooth_motor_driver_;
        sabertooth_motor_driver_ = nullptr;
    }
}

int SabertoothPropulsionDriver::sabertoothScale(double variable, double limit)
{ 
    if(variable > 0.0)
    {
        return (int)(max_command_value_ * (std::min(variable, limit) / limit));
    } 
    else if(variable < 0.0)
    {
        return (int)(max_command_value_ * (std::max(variable, -limit) / -limit)); 
    } 
    
    return 0;
}

void SabertoothPropulsionDriver::fillMixedModePacket(double motor_value, double max_value, double min_value, int positive_command, int negative_command, uint8_t* packet)
{
    const uint8_t undefined_command = 20;

    uint8_t command = undefined_command;
    uint8_t value = 0;

    //Check if the absolute value is larger than min value and accordingly choose the correct command.
    if(motor_value > min_value)
    {
        command = positive_command;
    }
    else if(motor_value < min_value)
    {
        command = negative_command;
    }

    // check if the value is larger than the min_value in the positive case and smaller in the negative case.
    if(command != undefined_command)
    {
        value = sabertoothScale(motor_value, max_value);
    } 
    else
    {
        command = positive_command;
        value = 0;
    }  

    sabertooth_motor_driver_->fillPacket(command, value, packet);
}

void SabertoothPropulsionDriver::fillRegularModePacket(double motor_value, double max_value, double min_value, int positive_command, int negative_command, uint8_t* packet)
{
    const uint8_t undefined_command = 20;

    uint8_t command = undefined_command;
    uint8_t value = 0;

    // Check if the absolute value is larger than min value and accordingly choose the correct command.
    if(motor_value > min_value)
    {
        command = positive_command;
    }
    else if(motor_value < min_value)
    {
        command = negative_command;
    }

    // Check if the value is larger than the min_value in the positive case and smaller in the negative case.
    if(command != undefined_command)
    {
        value = sabertoothScale(motor_value, max_value);
    } 
    else
    {
        command = positive_command;
        value = 0;
    }  

    // Fill the packet
    sabertooth_motor_driver_->fillPacket(command, value, packet);
}

/**
 *\brief Propulsion Create an array of straight command and turn command, using the twist message to translate from a twist message to a Sabertooth command array.
 *\param uint8_t* straightCommand
 *\param uint8_t* turnCommand
 *\param const geometry_msgs::Twist::ConstPtr& twist
 */
void SabertoothPropulsionDriver::processTwist(const geometry_msgs::Twist::ConstPtr& twist_message, uint8_t* straight_packet, uint8_t* turn_packet)
{

    fillMixedModePacket(twist_message->linear.x, maximum_x_value_, minimum_x_value_, SabertoothCommand::MixedForward, SabertoothCommand::MixedBackward, straight_packet);

    fillMixedModePacket(twist_message->angular.z, maximum_theta_value_, minimum_theta_value_, SabertoothCommand::MixedLeft, SabertoothCommand::MixedRight, turn_packet);

}

void SabertoothPropulsionDriver::processLeft(const std_msgs::Float32::ConstPtr& left_message, uint8_t* left_packet)
{
    ROS_INFO("LEFT %f ", left_message->data); 
    fillRegularModePacket(left_message->data, maximum_motor_value_, minimum_motor_value_, SabertoothCommand::Motor1Forward, SabertoothCommand::Motor1Backward, left_packet);
}

void SabertoothPropulsionDriver::processRight(const std_msgs::Float32::ConstPtr& right_message, uint8_t* right_packet)
{	
    ROS_INFO("RIGHT %f ", right_message->data);
    fillRegularModePacket(right_message->data, maximum_motor_value_, minimum_motor_value_, SabertoothCommand::Motor2Forward, SabertoothCommand::Motor2Backward, right_packet);
}
