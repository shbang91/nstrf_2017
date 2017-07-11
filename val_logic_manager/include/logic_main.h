#ifndef LOGICMAIN_H
#define LOGICMAIN_H

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/InteractiveMarkerInit.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_broadcaster.h>

#include "robot_state.h"

class LogicManager{
public:
    // Declare Node Handle
    ros::NodeHandle 					nh;
	ros::Publisher						val_ik_initpose_robot_joint_states_pub;	
	ros::Publisher						val_ik_finalpose_robot_joint_states_pub;	
	ros::Publisher						marker_pub;		

	ros::Subscriber 					interactive_marker_sub;
	ros::Subscriber 					operator_command_sub;	

	RobotState 							current_robot_state;	
	RobotState 							ik_init_robot_state;	
	RobotState 							ik_final_robot_state;	

	visualization_msgs::Marker			sample_object_marker;


	void init_sample_object_marker(); //initialize sample_marker
	void update_current_robot_state();
	
	void publish_robot_state_viz(); // the current robot state
	void publish_ik_init_state_viz(); // the initial IK pose
	void publish_ik_traj_state_viz(); // the IK trajectory
	void publish_ik_final_state_viz(); // the final IK pose


	// Node Callbacks
	void  interactive_callback(const visualization_msgs::InteractiveMarkerInitConstPtr& msg);
	void  operator_command_callback(const std_msgs::StringConstPtr& msg);


	void loop();

	LogicManager();
	~LogicManager();	
};

// Declare Existence of Global Variables
extern boost::mutex 							state_mutex;

extern sensor_msgs::JointState					global_joint_state_msg;
extern nav_msgs::Odometry 						global_odom_msg;
extern bool 									global_state_update_received;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::JointState, nav_msgs::Odometry> JointOdomSyncPolicy;

void  interactive_callback(const visualization_msgs::InteractiveMarkerInitConstPtr& msg);
void  operator_command_callback(const std_msgs::StringConstPtr& msg);

// Declare Global Funcs
void  stateFiltersCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg, const nav_msgs::OdometryConstPtr& odom_msg);


#endif