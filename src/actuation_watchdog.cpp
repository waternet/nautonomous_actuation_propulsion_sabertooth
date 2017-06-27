#include <nautonomous_actuation_propulsion_sabertooth/actuation_watchdog.h>

ActuationWatchdog::ActuationWatchdog(ros::NodeHandle node_handle){
    //Initialize status message
    status_msg_.level = -1;
    status_msg_.message = "Unknown";

    //Publisher
    watchdog_publisher_ = node_handle.advertise<diagnostic_msgs::DiagnosticStatus>("/actuation/propulsion/watchdog", 1000);

}

bool ActuationWatchdog::checkStatus(std::string response)
{
    diagnostic_msgs::DiagnosticStatus temp_status_msg = createStatus(response);

    if(adjustStatus(temp_status_msg))
    {
        watchdog_publisher_.publish(status_msg_); 
    } 
}

diagnostic_msgs::DiagnosticStatus ActuationWatchdog::createStatus(std::string response)
{
    diagnostic_msgs::DiagnosticStatus temp_status_msg;

    if(!response.empty())
    {
        temp_status_msg.level = 0; // OK status
        temp_status_msg.message = "Ok";    
    }
    else
    {
        temp_status_msg.level = 1; // WARN status
        temp_status_msg.message = "Error: no hearthbeat";
    }

    return temp_status_msg;
}

bool ActuationWatchdog::adjustStatus(diagnostic_msgs::DiagnosticStatus temp_status_msg){
    
    //Publish only new status
    if(temp_status_msg.level != status_msg_.level)
    {
        status_msg_.level = temp_status_msg.level;
        status_msg_.message = temp_status_msg.message;

        return true;
    }
    return false;
}

bool ActuationWatchdog::isActive()
{
    return (status_msg_.level == 0);
}

void ActuationWatchdog::setNotConnectedStatus()
{
    status_msg_.level = 2;
    status_msg_.message = "Not connected";
}