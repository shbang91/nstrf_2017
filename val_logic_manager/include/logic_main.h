#ifndef LOGICMAIN_H
#define LOGICMAIN_H

#include <ros/ros.h>

#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


class LogicManager{
public:
	ros::Publisher						example_pub;	
 	sensor_msgs::JointState 			current_joint_state;
	nav_msgs::Odometry 					current_robot_pose;	

	sensor_msgs::JointState 			ik_init_joint_state;
	nav_msgs::Odometry 					ik_init_robot_pose;	

	visualization_msgs::Marker			sample_object_marker;

	void init_sample_object_marker(); //initialize sample_marker
	
	void publish_robot_state_viz(); // the current robot state
	void publish_ik_traj_state_viz(); // the IK trajectory
	void publish_ik_final_state_viz(); // the final expected IK pose

	void loop();

	LogicManager();
	~LogicManager();	
};

// Declare Existence of Global Variables
extern LogicManager logic_manager;
extern boost::mutex state_mutex;


typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState, nav_msgs::Odometry> JointOdomSyncPolicy;
// Declare Global Funcs
void stateFiltersCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg, const nav_msgs::OdometryConstPtr& odom_msg);



#endif