#include "logic_main.h"

void  LogicManager::interactive_callback(const visualization_msgs::InteractiveMarkerInitConstPtr& msg){
	ROS_INFO("Logic Manager Interactive Marker Callback received");	
	// geometry_msgs::Pose marker_pose;
	// getIKSol(marker_pose);
	val_ik::DrakeOneHandSingleIk single_ik_srv;

	// Prepare initial IK robot state
    ik_init_robot_state.joint_state = current_robot_state.joint_state;
    ik_init_robot_state.robot_pose = current_robot_state.robot_pose;


    // Prepare Robot State msg
    val_ik_msgs::RobotState robot_state_msg;
    geometry_msgs::Pose robot_pose;
    // Convert odometry message to geometry_msgs::Pose
    robot_pose = ik_init_robot_state.robot_pose.pose.pose; // This is because robot_pose is a nav_msgs::Odometry
    robot_state_msg.robot_pose = robot_pose;
	robot_state_msg.body_joint_states = ik_init_robot_state.joint_state;    


	single_ik_srv.request.robot_state = robot_state_msg;

	if (single_ik_client.call(single_ik_srv)){
        ROS_INFO("Single IK Call Successful");
        //ik_srv.response.robot_joint_states.floating_joint_states;
        //ik_srv.response.robot_joint_states.body_joint_states;
    }
    else{
       ROS_ERROR("Failed to call Single IK service");
    }

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