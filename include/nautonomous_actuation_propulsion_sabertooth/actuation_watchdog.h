#ifndef ACTUATIONWATCHDOG_H_
#define ACTUATIONWATCHDOG_H_

#include <ros/ros.h>

#include <diagnostic_msgs/DiagnosticStatus.h>

class ActuationWatchdog
{
    private: 
        ros::Publisher watchdog_publisher_;
        diagnostic_msgs::DiagnosticStatus status_msg_;

        diagnostic_msgs::DiagnosticStatus createStatus(std::string response);

        bool adjustStatus(diagnostic_msgs::DiagnosticStatus temp_status_msg);

    public:
        ActuationWatchdog(ros::NodeHandle node_handle);

        bool checkStatus(std::string response);

        bool isActive();

        void setNotConnectedStatus();

};

#endif /* ACTUATIONWATCHDOG_H_ */