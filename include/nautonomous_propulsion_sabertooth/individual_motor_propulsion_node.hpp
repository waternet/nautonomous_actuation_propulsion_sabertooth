#include <ros/ros.h>

#include <nautonomous_propulsion_msgs/IndividualMotorPropulsion.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>

class IndividualMotorPropulsionPublisher {
 public:
  IndividualMotorPropulsionPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Subscriber callbacks
  void callbackNextPropulsionMode(const std_msgs::Bool& msg);
  void callbackSetPropulsionMode(const std_msgs::Int8& msg);

  void callbackNextDryDockMode(const std_msgs::Bool& msg);
  void callbackSetDryDockMode(const std_msgs::Int8& msg);

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

int main(int argc, char **argv);
