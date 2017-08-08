#ifndef VALIKIHMCBRIDGE_H
#define VALIKIHMCBRIDGE_H

#include <ros/ros.h>
#include "robot_state.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

//IHMC Control Messages
#include "ihmc_msgs/WholeBodyTrajectoryRosMessage.h"

// Drake FK Service
#include "val_ik/DrakeFKBodyPose.h"
#include "val_ik/DrakeIKVal.h"
#include "val_ik/DrakeOneHandSingleIk.h"

#include "compile_settings.h"
//#define ON_REAL_ROBOT

// robot_side enum
#define LEFT_HAND 0
#define RIGHT_HAND 1


class IK_IHMC_Bridge{
public:
	ros::ServiceClient  				ik_client;
	ros::ServiceClient  				single_ik_client;	
	ros::ServiceClient 					fk_client;

	geometry_msgs::Pose 				initial_left_foot_pose;
	geometry_msgs::Pose 				initial_right_foot_pose;
	geometry_msgs::Pose 				initial_pelvis_pose;

	RobotState 							ik_init_robot_state;
	RobotState 							ik_final_robot_state;	

	void set_init_IK_state(RobotState &start_state);
	void set_final_IK_state(RobotState &end_state);	

	const std::vector<std::string> 			rarm_joint_names;
	const std::vector<std::string>			larm_joint_names;

	const std::vector<std::string> 			nasa_left_arm_joint_names;
	const std::vector<std::string>			nasa_right_arm_joint_names;


	// Calculates the IK solution with the hand pose as the only constraint.
	bool calc_single_hand_IK(const geometry_msgs::Pose& des_hand_pose, const int& robot_side, 
							 const RobotState& robot_state_input, RobotState& robot_state_output);	

	// Finds the body poses of a given robot state using Drake's Forward Kinematics
	bool FK_bodies(	RobotState &robot_state,
				    std::vector<std::string> &body_queries, std::vector<geometry_msgs::Pose> &body_poses);

	bool prepareSingleIKWBC(RobotState &start_state, RobotState &end_state, double &traj_time,
						 ihmc_msgs::WholeBodyTrajectoryRosMessage &wbc_traj_msg,     
						 sensor_msgs::JointState &left_arm_msg,	
						 sensor_msgs::JointState &right_arm_msg,
						 int left_hand_open_close_status,
						 int right_hand_open_close_status);

	IK_IHMC_Bridge();
	~IK_IHMC_Bridge();	

};

#endif