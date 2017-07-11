#ifndef LOGICMAIN_H
#define LOGICMAIN_H

#include <ros/ros.h>

#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class LogicManager{
public:
	sensor_msgs::JointState 	current_joint_state;
	nav_msgs::Odometry 			current_robot_pose;	

	LogicManager();
	~LogicManager();	
};

// Declare Existence of Global Variables
extern LogicManager logic_manager;
extern boost::mutex state_mutex;


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState, nav_msgs::Odometry> MySyncPolicy;
// Declare Global Funcs
void stateFiltersCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg, const nav_msgs::OdometryConstPtr& odom_msg);



#endif