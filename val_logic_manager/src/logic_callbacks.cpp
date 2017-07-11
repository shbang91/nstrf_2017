#include "logic_main.h"
void stateFiltersCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg, const nav_msgs::OdometryConstPtr& odom_msg){
	ROS_INFO("Callback received");
}