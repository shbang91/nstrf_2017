#include "logic_main.h"

void  LogicManager::interactive_callback(const visualization_msgs::InteractiveMarkerInitConstPtr& msg){
	ROS_INFO("Logic Manager Interactive Marker Callback received");	
	// geometry_msgs::Pose marker_pose;
	// getIKSol(marker_pose);

}

void  LogicManager::operator_command_callback(const std_msgs::StringConstPtr& msg){
	ROS_INFO("Logic Manager Operator Command Callback");	
}





void stateFiltersCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg, const nav_msgs::OdometryConstPtr& odom_msg){
	state_mutex.lock();
		global_joint_state_msg = (*joint_state_msg);
		global_odom_msg = (*odom_msg);
		global_first_state_update_received = true;
		global_state_update_received = true;		
	state_mutex.unlock();
}