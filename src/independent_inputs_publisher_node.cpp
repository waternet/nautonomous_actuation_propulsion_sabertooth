/*
 * Author: Antoine Seewer
 */

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nautonomous_msgs/IndependentInputs.h>

class IndependentInputsPublisher {
 public:
  IndependentInputsPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Subscriber callbacks
  void cbChangeInputs(const std_msgs::Bool& msg);

//  // Timer callbacks
//  void cbStep(const ros::TimerEvent&);

 private:
  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Subscribers
  ros::Subscriber sub_change_inputs_;

  // Publishers
  ros::Publisher pub_independent_inputs_;

//  // Timers
//  ros::Timer timer_;

  // Independent inputs
  nautonomous_msgs::IndependentInputs independent_inputs_;
  double left_input_array_[12] = {1, -1, 0.5, -0.5, 1, -1, 0.5, -0.5, 1, -0.5, 0.5, -0.25};
  double right_input_array_[12] = {1, -1, 0.5, -0.5, -1, 1, -0.5, 0.5, -0.5, 1, -0.25, 0.5};

  // Settings
  int i_;
};

IndependentInputsPublisher::IndependentInputsPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      i_(0) {
  // Subscribers
  // use "rostopic pub -1 /change_inputs std_msgs/Bool 1" to publish on this topic from the terminal...
  sub_change_inputs_ = nh_.subscribe("/change_inputs", 1, &IndependentInputsPublisher::cbChangeInputs, this);

  // Publishers
  pub_independent_inputs_ = nh_private_.advertise<nautonomous_msgs::IndependentInputs>("independent_inputs", 1, true);

//  // Timers
//  timer_ = nh_.createTimer(ros::Duration(0.01), &IndependentInputsPublisher::cbStep, this);
}

void IndependentInputsPublisher::cbChangeInputs(const std_msgs::Bool& msg) {

  ROS_INFO("Got input");

  if (i_ < 12) {
    // Retrieve next independent inputs
    independent_inputs_.left_motor_input = left_input_array_[i_];
    independent_inputs_.right_motor_input = right_input_array_[i_];

    // Publish independent inputs
    pub_independent_inputs_.publish(independent_inputs_);
  }
  i_++;
}

//void IndependentInputsPublisher::cbStep(const ros::TimerEvent&) {
//  independent_inputs_.left_motor_input = 0.5;
//  independent_inputs_.right_motor_input = -0.5;
//
//  // Publish independent inputs
//  pub_independent_inputs_.publish(independent_inputs_);
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "independent_inputs_publisher");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  IndependentInputsPublisher independent_inputs_publisher(nh, nh_private);

  ROS_INFO("Independent inputs publisher node initialized.");
  ros::spin();
}
