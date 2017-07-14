#ifndef VALIKIHMCBRIDGE_H
#define VALIKIHMCBRIDGE_H

#include <ros/ros.h>
#include "robot_state.h"
#include "geometry_msgs/Pose.h"

//IHMC Control Messages
#include "ihmc_msgs/WholeBodyTrajectoryRosMessage.h"

class IK_IHMC_Bridge{
public:
	geometry_msgs::Pose 				initial_left_foot_pose;
	geometry_msgs::Pose 				initial_right_foot_pose;
	geometry_msgs::Pose 				initial_pelvis_pose;

	RobotState 							ik_init_robot_state;
	RobotState 							ik_final_robot_state;	
	void calc_single_hand_IK(geometry_msgs::Pose &des_hand_pose);	
	void set_init_IK_state(RobotState &start_state);
	void set_final_IK_state(RobotState &end_state);	

	const std::vector<std::string> 			rarm_joint_names;
	const std::vector<std::string>			larm_joint_names;


	bool prepareSingleIKWBC(RobotState &start_state, RobotState &end_state, double &traj_time,
						 ihmc_msgs::WholeBodyTrajectoryRosMessage &wbc_traj_msg);

	IK_IHMC_Bridge();
	~IK_IHMC_Bridge();	

};

#endif