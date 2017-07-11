#include "logic_main.h"
void stateFiltersCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg, const nav_msgs::OdometryConstPtr& odom_msg){
	ROS_INFO("Callback received");
	state_mutex.lock();
		logic_manager.current_joint_state = (*joint_state_msg);
		logic_manager.current_robot_pose = (*odom_msg);	
	state_mutex.unlock();

}
