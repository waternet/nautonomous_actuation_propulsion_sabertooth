#include <nautonomous_propulsion_sabertooth/IndividualMotorPropulsionPublisher.h>

class IndividualMotorPropulsionPublisher {
 public:
  IndividualMotorPropulsionPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

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

IndividualMotorPropulsionPublisher::IndividualMotorPropulsionPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      propulsion_mode_index_(0),
      dry_dock_mode_index_(0) {
  // Subscribers
  // use "rostopic pub -1 /change_inputs std_msgs/Bool 1" to publish on this topic from the terminal...
  sub_propulsion_next_mode_ = nh_.subscribe("/propulsion/sabertooth/next_mode", 1, &IndividualMotorPropulsionPublisher::callbackNextPropulsionMode, this);
  sub_propulsion_set_mode_ = nh_.subscribe("/propulsion/sabertooth/set_mode", 1, &IndividualMotorPropulsionPublisher::callbackSetPropulsionMode, this);
  
  sub_dry_dock_next_mode_ = nh_.subscribe("/motor_mode/dry_dock/next_mode", 1, &IndividualMotorPropulsionPublisher::callbackNextDryDockMode, this);
  sub_dry_dock_set_mode_ = nh_.subscribe("/motor_mode/dry_dock/set_mode", 1, &IndividualMotorPropulsionPublisher::callbackSetDryDockMode, this);

  // Publishers
  pub_independent_inputs_ = nh_private_.advertise<nautonomous_propulsion_msgs::IndividualPropulsionMotor>("/motor_mode/independent_inputs", 1, true);

//  // Timers
//  timer_ = nh_.createTimer(ros::Duration(0.01), &IndividualMotorPropulsionPublisher::callbackStep, this);
}

void IndividualMotorPropulsionPublisher::callbackNextPropulsionMode(const std_msgs::Bool& msg) {
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

void IndividualMotorPropulsionPublisher::callbackSetPropulsionMode(const std_msgs::Int8& msg) {
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



void IndividualMotorPropulsionPublisher::callbackNextDryDockMode(const std_msgs::Bool& msg) {
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

void IndividualMotorPropulsionPublisher::callbackSetDryDockMode(const std_msgs::Int8& msg) {
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

  IndividualMotorPropulsionPublisher independent_inputs_publisher(nh, nh_private);

  ROS_INFO("Independent inputs publisher node initialized.");
  ros::spin();
}
