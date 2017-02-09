#include "../include/nautonomous_actuation_simulator/propulsion_simulator_node.hpp"

/*
 * If the propulsion topic arrives, read message and set the velocity x, y, th for the robot.
 */
void propulsionCallback(const geometry_msgs::Twist::ConstPtr& twist) {
	velocity.linear.x = twist->linear.x;
	velocity.linear.y = twist->linear.y;
	velocity.angular.z = twist->angular.z;
}

/*
 * Updates the map, with the grid and meta data from the message
 */
// void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& occupancy) {
// 	nautonomous_occupancy_grid.data = occupancy->data;
// 	nautonomous_occupancy_grid.header = occupancy->header;
// 	nautonomous_occupancy_grid.info = occupancy->info;
// }

void calculatePosition(){
	current_time = ros::Time::now();
	delta_time = (current_time - last_time).toSec();

	delta.linear.x = (velocity.linear.x * cos(position.angular.z) - velocity.linear.y * sin(position.angular.z)) * delta_time;
	delta.linear.y = (velocity.linear.x * sin(position.angular.z) + velocity.linear.y * cos(position.angular.z)) * delta_time;
	delta.angular.z = position.angular.z * delta_time;

	position.linear.x += delta.linear.x;
	position.linear.y += delta.linear.y;
	position.angular.z += delta.angular.z;

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	odom_quat = tf::createQuaternionMsgFromYaw(position.angular.z);
}

// Implement check if the simulator does not cross the border of the map, commented out since it cannot be tested currently.
// void calculateMapIndex(){
// 	if(nautonomous_occupancy_grid.info.height){
// 		int index_x = nautonomous_occupancy_grid.info.height
// 			- (abs(x + nautonomous_occupancy_grid.info.origin.position.x)
// 					/ nautonomous_occupancy_grid.info.resolution);
// 		int index_y = ((y - nautonomous_occupancy_grid.info.origin.position.y)
// 			/ nautonomous_occupancy_grid.info.resolution);
// 		//ROS_INFO("(%4.2f, %4.2f | %4.2f [%4.2f]) is index (%d, %d)", x, y, th, degrees, index_x, index_y);
// 		int index = nautonomous_occupancy_grid.info.height * index_y + index_x;
// 		if (index >= 0 && index <= (nautonomous_occupancy_grid.info.height*nautonomous_occupancy_grid.info.width)) {
// 			int value = nautonomous_occupancy_grid.data[index];
// 			//ROS_INFO("index %d value %d", index, value);
// 			if (value > 50) {
// 				//reverse action
// 				ROS_INFO("REVERSE");
// 				x -= delta_x;
// 				y -= delta_y;
// 				th -= delta_th;
// 			}
// 		}
// 	}
// }

void publishTransform(tf::TransformBroadcaster transform_broadcaster){
	geometry_msgs::TransformStamped odom_trans;

	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom_combined";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = position.linear.x;
	odom_trans.transform.translation.y = position.linear.y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	transform_broadcaster.sendTransform(odom_trans);
}

void publishOdometry(){
	nav_msgs::Odometry odom;

	odom.header.stamp = current_time; 
	odom.header.frame_id = "odom_combined"; //odom_combined

	//set the position
	odom.pose.pose.position.x = position.linear.x;
	odom.pose.pose.position.y = position.linear.y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = "base_link"; //base_link
	odom.twist.twist.linear.x = velocity.linear.x;
	odom.twist.twist.linear.y = velocity.linear.y;
	odom.twist.twist.angular.z = velocity.angular.z;

	//publish the message
	odom_pub.publish(odom);
}

void simulatePropulsion(ros::NodeHandle n, tf::TransformBroadcaster transform_broadcaster){
	ros::Rate r(25);
	while (n.ok()) {

		ros::spinOnce();              	 	// check for incoming messages

		calculatePosition();

		//Implement check if the simulator does not cross the border of the map, commented out since it cannot be tested currently.
		//calculateMapIndex();

		publishTransform(transform_broadcaster);

		publishOdometry();

		last_time = current_time;
		r.sleep();
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "propulsion_simulation_node");

	ros::NodeHandle n;

	ros::NodeHandle param("~");
    param.param("start_x", position.linear.x, 0.0);
	param.param("start_y", position.linear.y, 0.0);
	param.param("start_theta", position.angular.z, 0.0);
	
	//Topics to subscribe to, multiplexed propulsion and the dimensions of the map.
	ros::Subscriber propulsionSubscriber = n.subscribe(
			"propulsion", 10, propulsionCallback);

	//ros::Subscriber mapSubscriber = n.subscribe("map", 10, mapCallback);

 	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

	last_time = ros::Time::now();

	tf::TransformBroadcaster transform_broadcaster;

    simulatePropulsion(n, transform_broadcaster);

	return 0;
}
