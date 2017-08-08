#ifndef ROBOTSTATE_H
#define ROBOTSTATE_H

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#define CLOSE_HAND 0
#define OPEN_HAND 1

#define CLOSE_HAND_VALS 2.9
#define OPEN_HAND_VALS 0.0

class RobotState{
public:
	std::string 						robot_namespace;
 	sensor_msgs::JointState 			joint_state;
	nav_msgs::Odometry 					robot_pose;
	bool								valid_fields;
    
	void getRPY(double &pelvis_roll, double &pelvis_pitch, double &pelvis_yaw);
	void getXYZ(double &pelvis_x, double &pelvis_y, double &pelvis_z);			
	void publish_viz(ros::Publisher &state_publisher, tf::TransformBroadcaster &br);	

	RobotState();
	~RobotState();
};

#endif