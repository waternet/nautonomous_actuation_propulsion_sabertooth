/*
 * Author: Antoine Seewer
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <nautonomous_msgs/IndependentInputs.h>

class IndependentInputsPublisher {
 public:
  IndependentInputsPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Subscriber callbacks
  void callbackNextPropulsionMode(const std_msgs::Bool& msg);
  void callbackSetPropulsionMode(const std_msgs::Int8& msg);

  void callbackNextDryDockMode(const std_msgs::Bool& msg);
  void callbackSetDryDockMode(const std_msgs::Int8& msg);
//  // Timer callbacks
//  void callbackStep(const ros::TimerEvent&);

 private:
  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers
  ros::Subscriber sub_propulsion_next_mode_;
  ros::Subscriber sub_propulsion_set_mode_;
  ros::Subscriber sub_dry_dock_next_mode_;
  ros::Subscriber sub_dry_dock_set_mode_;

  // Publishers
  ros::Publisher pub_independent_inputs_;

//  // Timers
//  ros::Timer timer_;
  // Independent inputs
  nautonomous_msgs::IndependentInputs independent_inputs_;
  double left_motor_propulsion_array_[13] =   {1.0, -1.0, 0.5, -0.5,  1.0, -1.0, 0.5, -0.5,  1.0, -0.5,  0.5, -0.25, 0.0};
  double right_motor_propulsion_array_[13] =  {1.0, -1.0, 0.5, -0.5, -1.0,  1.0, -0.5, 0.5, -0.5,  1.0, -0.25, 0.5, 0.0};

  double left_motor_dry_dock_array_[13] =   {0.1, -0.1, 0.05, -0.05,  0.1, -0.1, 0.05, -0.05,  0.1, -0.05,  0.05, -0.025, 0.0};
  double right_motor_dry_dock_array_[13] =  {0.1, -0.1, 0.05, -0.05, -0.1,  0.1, -0.05, 0.05, -0.05,  0.1, -0.025, 0.05, 0.0};

  // Settings
  int propulsion_mode_index_;
  int dry_dock_mode_index_;
};

IndependentInputsPublisher::IndependentInputsPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      propulsion_mode_index_(0),
      dry_dock_mode_index_(0) {
  // Subscribers
  // use "rostopic pub -1 /change_inputs std_msgs/Bool 1" to publish on this topic from the terminal...
  sub_propulsion_next_mode_ = nh_.subscribe("/motor_mode/propulsion/next_mode", 1, &IndependentInputsPublisher::callbackNextPropulsionMode, this);
  sub_propulsion_set_mode_ = nh_.subscribe("/motor_mode/propulsion/set_mode", 1, &IndependentInputsPublisher::callbackSetPropulsionMode, this);
  
  sub_dry_dock_next_mode_ = nh_.subscribe("/motor_mode/dry_dock/next_mode", 1, &IndependentInputsPublisher::callbackNextDryDockMode, this);
  sub_dry_dock_set_mode_ = nh_.subscribe("/motor_mode/dry_dock/set_mode", 1, &IndependentInputsPublisher::callbackSetDryDockMode, this);

  // Publishers
  pub_independent_inputs_ = nh_private_.advertise<nautonomous_msgs::IndependentInputs>("/motor_mode/independent_inputs", 1, true);

//  // Timers
//  timer_ = nh_.createTimer(ros::Duration(0.01), &IndependentInputsPublisher::callbackStep, this);
}

void IndependentInputsPublisher::callbackNextPropulsionMode(const std_msgs::Bool& msg) {
  if (!msg.data){
    ROS_INFO("Message content is false, so do not give next input.");
    return;
  }
  if (propulsion_mode_index_ < 13) {
    // Retrieve next independent inputs
    independent_inputs_.left_motor_input = left_motor_propulsion_array_[propulsion_mode_index_];
    independent_inputs_.right_motor_input = right_motor_propulsion_array_[propulsion_mode_index_];

    // Publish independent inputs
    pub_independent_inputs_.publish(independent_inputs_);
  } 
  propulsion_mode_index_ = (propulsion_mode_index_ + 1) % 13;
}

void IndependentInputsPublisher::callbackSetPropulsionMode(const std_msgs::Int8& msg) {
  if (msg.data < 13) {
    // Retrieve next independent inputs
    independent_inputs_.left_motor_input = left_motor_propulsion_array_[msg.data];
    independent_inputs_.right_motor_input = right_motor_propulsion_array_[msg.data];

    // Publish independent inputs
    pub_independent_inputs_.publish(independent_inputs_);
  } else {
    ROS_INFO("Message data exceeds size of motor states.");
  }
}



void IndependentInputsPublisher::callbackNextDryDockMode(const std_msgs::Bool& msg) {
  if (!msg.data){
    ROS_INFO("Message content is false, so do not give next input.");
    return;
  }
  if (dry_dock_mode_index_ < 13) {
    // Retrieve next independent inputs
    independent_inputs_.left_motor_input = left_motor_dry_dock_array_[dry_dock_mode_index_];
    independent_inputs_.right_motor_input = right_motor_dry_dock_array_[dry_dock_mode_index_];

    // Publish independent inputs
    pub_independent_inputs_.publish(independent_inputs_);
  } 
  dry_dock_mode_index_ = (dry_dock_mode_index_ + 1) % 13;
}

void IndependentInputsPublisher::callbackSetDryDockMode(const std_msgs::Int8& msg) {
  if (msg.data < 13) {
    // Retrieve next independent inputs
    independent_inputs_.left_motor_input = left_motor_dry_dock_array_[msg.data];
    independent_inputs_.right_motor_input = right_motor_dry_dock_array_[msg.data];

    // Publish independent inputs
    pub_independent_inputs_.publish(independent_inputs_);
  } else {
    ROS_INFO("Message data exceeds size of motor states.");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "independent_inputs_publisher");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  IndependentInputsPublisher independent_inputs_publisher(nh, nh_private);

  ROS_INFO("Independent inputs publisher node initialized.");
  ros::spin();
}
