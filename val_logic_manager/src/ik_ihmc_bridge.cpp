#include "ik_ihmc_bridge.h"

IK_IHMC_Bridge::IK_IHMC_Bridge() : rarm_joint_names{"rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", 
													"rightElbowPitch", "rightForearmYaw", "rightWristRoll", "rightWristPitch"}, 
								   larm_joint_names{"leftShoulderPitch",  "leftShoulderRoll", "leftShoulderYaw",
													"leftElbowPitch", "leftForearmYaw", "leftWristRoll", "leftWristPitch" }
{


} // End Constructor

IK_IHMC_Bridge::~IK_IHMC_Bridge(){} // End Destructor


void IK_IHMC_Bridge::set_init_IK_state(RobotState &start_state){
	ik_init_robot_state.robot_pose = start_state.robot_pose;
	ik_init_robot_state.joint_state = start_state.joint_state;
	ik_init_robot_state.valid_fields = start_state.valid_fields;
}
void IK_IHMC_Bridge::set_final_IK_state(RobotState &end_state){
	ik_final_robot_state.robot_pose = end_state.robot_pose;		
	ik_final_robot_state.joint_state = end_state.joint_state;	
	ik_final_robot_state.valid_fields = end_state.valid_fields;
}



bool IK_IHMC_Bridge::prepareSingleIKWBC(RobotState &start_state, RobotState &end_state, double &traj_time,
									 ihmc_msgs::WholeBodyTrajectoryRosMessage &wbc_traj_msg){
	// Initialize ik positions
	set_init_IK_state(start_state);
	set_final_IK_state(end_state);	
	std::vector<std::string>::iterator it;

	int traj_unique_id = 1;

	if (!start_state.valid_fields){
		ROS_ERROR("Starting state passed is not valid");
		return false;
	}

	if (!end_state.valid_fields){
		ROS_ERROR("Ending state passed is not valid");
		return false;
	}	

	if (ik_init_robot_state.valid_fields && ik_final_robot_state.valid_fields){

	    // Begin Right Arm Trajectory Message ----------------------------------------------------------------------------------------------
	    ihmc_msgs::ArmTrajectoryRosMessage rarm_traj_msg;
		// Right Arm: Add initial and final points
		for (size_t i = 0; i < rarm_joint_names.size(); i++){
			// Begin right arm joint;
	        ihmc_msgs::OneDoFJointTrajectoryRosMessage rarm_joint;

			// Find Joint Index of Right Arm Joints for starting waypoint
			it = std::find (ik_init_robot_state.joint_state.name.begin(), ik_init_robot_state.joint_state.name.end(), rarm_joint_names.at(i));
			int sw_joint_index = std::distance(ik_init_robot_state.joint_state.name.begin(), it);
			// Set the joint values
			double joint_start_value = ik_init_robot_state.joint_state.position[sw_joint_index];


			// Find Joint Index of Right Arm Joints for ending waypoint
			it = std::find (ik_final_robot_state.joint_state.name.begin(), ik_final_robot_state.joint_state.name.end(), rarm_joint_names.at(i));
			int ew_joint_index = std::distance(ik_final_robot_state.joint_state.name.begin(), it);		
			double joint_end_value   = ik_final_robot_state.joint_state.position[ew_joint_index];

			std::cout << "SW Found joint " << ik_init_robot_state.joint_state.name[sw_joint_index]  << " val: " << joint_start_value << std::endl; 
			std::cout << "EW Found joint " << ik_final_robot_state.joint_state.name[ew_joint_index]  << " val: " << joint_end_value << std::endl; 			

            ihmc_msgs::TrajectoryPoint1DRosMessage joint_start_val_msg;
            ihmc_msgs::TrajectoryPoint1DRosMessage joint_end_val_msg; 
		
            joint_start_val_msg.time = 0.0;    
            joint_start_val_msg.position = joint_start_value;  
            joint_start_val_msg.velocity = 0.0; 
            joint_start_val_msg.unique_id = traj_unique_id;

            joint_end_val_msg.time = traj_time;      
            joint_end_val_msg.position = joint_end_value;      
            joint_end_val_msg.velocity = 0.0;            
            joint_end_val_msg.unique_id = traj_unique_id;

			rarm_joint.trajectory_points.push_back(joint_start_val_msg);
			rarm_joint.trajectory_points.push_back(joint_end_val_msg);
			rarm_joint.unique_id = traj_unique_id;


			rarm_traj_msg.robot_side = rarm_traj_msg.RIGHT; // RIGHT SIDE		
			rarm_traj_msg.joint_trajectory_messages.push_back(rarm_joint);
			rarm_traj_msg.execution_mode = 0;
			rarm_traj_msg.previous_message_id = 0;
			rarm_traj_msg.unique_id = traj_unique_id;

		}


	    // Begin Left Arm Trajectory Message ----------------------------------------------------------------------------------------------
	    ihmc_msgs::ArmTrajectoryRosMessage larm_traj_msg;
		// Right Arm: Add initial and final points
		for (size_t i = 0; i < larm_joint_names.size(); i++){
			// Begin right arm joint;
	        ihmc_msgs::OneDoFJointTrajectoryRosMessage larm_joint;

			// Find Joint Index of Left Arm Joints for starting waypoint
			it = std::find (ik_init_robot_state.joint_state.name.begin(), ik_init_robot_state.joint_state.name.end(), larm_joint_names.at(i));
			int sw_joint_index = std::distance(ik_init_robot_state.joint_state.name.begin(), it);
			// Set the joint values
			double joint_start_value = ik_init_robot_state.joint_state.position[sw_joint_index];


			// Find Joint Index of Leftt Arm Joints for ending waypoint
			it = std::find (ik_final_robot_state.joint_state.name.begin(), ik_final_robot_state.joint_state.name.end(), larm_joint_names.at(i));
			int ew_joint_index = std::distance(ik_final_robot_state.joint_state.name.begin(), it);		
			double joint_end_value   = ik_final_robot_state.joint_state.position[ew_joint_index];

            ihmc_msgs::TrajectoryPoint1DRosMessage joint_start_val_msg;
            ihmc_msgs::TrajectoryPoint1DRosMessage joint_end_val_msg; 


		
            joint_start_val_msg.time = 0.0;    
            joint_start_val_msg.position = joint_start_value;  
            joint_start_val_msg.velocity = 0.0; 
            joint_start_val_msg.unique_id = traj_unique_id;

            joint_end_val_msg.time = traj_time;      
            joint_end_val_msg.position = joint_end_value;      
            joint_end_val_msg.velocity = 0.0;            
            joint_end_val_msg.unique_id = traj_unique_id;

			larm_joint.trajectory_points.push_back(joint_start_val_msg);
			larm_joint.trajectory_points.push_back(joint_end_val_msg);
			larm_joint.unique_id = traj_unique_id;

			larm_traj_msg.robot_side = larm_traj_msg.LEFT; // LEFT SIDE		
			larm_traj_msg.joint_trajectory_messages.push_back(larm_joint);
			larm_traj_msg.execution_mode = 0;
			larm_traj_msg.previous_message_id = 0;
			larm_traj_msg.unique_id = traj_unique_id;
		}		

		wbc_traj_msg.right_arm_trajectory_message = rarm_traj_msg;		
		wbc_traj_msg.left_arm_trajectory_message = larm_traj_msg;
		wbc_traj_msg.unique_id = traj_unique_id;

		wbc_traj_msg.right_foot_trajectory_message.robot_side = 1;
		wbc_traj_msg.right_hand_trajectory_message.robot_side = 1;

	}

/*
    ihmc_msgs::WholeBodyTrajectoryRosMessage wbc_traj_msg;

	joint_index    ik_init_robot_state.joint_state.name


        ihmc_msgs::OneDoFJointTrajectoryRosMessage rarm_joint;
        // for each right arm joint:
            ihmc_msgs::TrajectoryPoint1DRosMessage start_joint_wp; 
            ihmc_msgs::TrajectoryPoint1DRosMessage end_joint_wp;

        rarm_traj_msg.unique_id = 1;
       
    // left arm

    // create chest trajectory_msg    
    ihmc_msgs::ChestTrajectoryRosMessage chest_trajectory_msg;
            ihmc_msgs::SO3TrajectoryPointRosMessage     start_SO3_chest_traj;
            ihmc_msgs::SO3TrajectoryPointRosMessage     end_SO3_chest_traj;            

        chest_trajectory_msg.taskspace_trajectory_points.push_back(start_SO3_chest_traj);
        chest_trajectory_msg.taskspace_trajectory_points.push_back(end_SO3_chest_traj);        

    ihmc_msgs::PelvisTrajectoryRosMessage pelvis_trajectory_message;  
           ihmc_msgs::SE3TrajectoryPointRosMessage      start_SE3_pelvis_traj;
           ihmc_msgs::SE3TrajectoryPointRosMessage      end_SE3_pelvis_traj;           

        pelvis_trajectory_message.taskspace_trajectory_points.push_back(start_SE3_pelvis_traj);
        pelvis_trajectory_message.taskspace_trajectory_points.push_back(end_SE3_pelvis_traj);        


    wbc_traj_msg.unique_id = 1;
*/
}