#include "logic_main.h"
LogicManager::LogicManager(){
	init_sample_object_marker();
    ik_init_robot_state.robot_namespace = "val_ik_initpose_robot";
    ik_final_robot_state.robot_namespace = "val_ik_finalpose_robot";
    righthand_grasp_index = 0;
    hand_to_use = RIGHT_HAND;
} // Constructor
LogicManager::~LogicManager(){} // Destructor



void LogicManager::publish_ik_init_state_viz(){
    if (ik_init_robot_state.valid_fields){
        ik_init_robot_state.publish_viz(val_ik_initpose_robot_joint_states_pub, br);    
    }
}

void LogicManager::publish_ik_final_state_viz(){
    if (ik_final_robot_state.valid_fields){
        ik_final_robot_state.publish_viz(val_ik_finalpose_robot_joint_states_pub, br);    
    }
}

void LogicManager::update_current_robot_state(){
    if (global_state_update_received){
        current_robot_state.joint_state = global_joint_state_msg;
        current_robot_state.robot_pose = global_odom_msg; 

        // Test
        ik_init_robot_state.joint_state = current_robot_state.joint_state;
        ik_init_robot_state.robot_pose = current_robot_state.robot_pose;
        ik_init_robot_state.valid_fields = true;
        publish_ik_init_state_viz();
        // Test


        ROS_INFO_THROTTLE(1, "Robot State Update Received");
        //ROS_INFO_THROTTLE(2, "JointState Size = %zu", global_joint_state_msg.position.size());
        global_state_update_received = false;
    }
}


void LogicManager::sendSingleIKWBC(){
    // Prepare Whole Body Trajectory Message
    double traj_time = 2.0;
    ihmc_msgs::WholeBodyTrajectoryRosMessage wbc_traj_msg;
    // Fill in fields:
    if (ik_manager.prepareSingleIKWBC(ik_init_robot_state, ik_final_robot_state, traj_time, wbc_traj_msg)){
        ROS_INFO("Single EndPoint WBC Message successfully prepared.");
        ROS_INFO("Sending Message");
        ihmc_wholebody_pub.publish(wbc_traj_msg);
    }

}

void LogicManager::loop(){
    update_current_robot_state();
    sample_object_marker.header.stamp = ros::Time::now();
    marker_pub.publish(sample_object_marker);

    publish_ik_init_state_viz();
    publish_ik_final_state_viz();

}

void LogicManager::sendWBCGoHome(){
    int GO_HOME_ID = 2;
    double go_home_time = 2.0;
    // Left Arm GO Home Message
    ihmc_msgs::GoHomeRosMessage larm_go_home_msg;
    larm_go_home_msg.body_part =  larm_go_home_msg.ARM;
    larm_go_home_msg.robot_side =  larm_go_home_msg.LEFT;
    larm_go_home_msg.trajectory_time = go_home_time;
    larm_go_home_msg.unique_id = GO_HOME_ID;
    GO_HOME_ID++;

    // Right Arm GO Home Message
    ihmc_msgs::GoHomeRosMessage rarm_go_home_msg;
    rarm_go_home_msg.body_part =  rarm_go_home_msg.ARM;
    rarm_go_home_msg.robot_side =  rarm_go_home_msg.RIGHT;
    rarm_go_home_msg.trajectory_time = go_home_time;
    rarm_go_home_msg.unique_id = GO_HOME_ID;
    GO_HOME_ID++;

    // Chest GO Home Message
    ihmc_msgs::GoHomeRosMessage chest_go_home_msg;
    chest_go_home_msg.body_part =  chest_go_home_msg.CHEST;
    chest_go_home_msg.robot_side =  0;
    chest_go_home_msg.trajectory_time = go_home_time;
    chest_go_home_msg.unique_id = GO_HOME_ID;
    GO_HOME_ID++;    

    // Pelvis GO Home Message
    ihmc_msgs::GoHomeRosMessage pelvis_go_home_msg;
    pelvis_go_home_msg.body_part =  pelvis_go_home_msg.PELVIS;
    pelvis_go_home_msg.robot_side =  0;
    pelvis_go_home_msg.trajectory_time = go_home_time;
    pelvis_go_home_msg.unique_id = GO_HOME_ID;

    ROS_INFO("Attempting to call FK for Feet and Pelvis pose");
    std::vector<std::string> body_queries;
    std::vector<geometry_msgs::Pose> body_poses;

    body_queries.push_back("leftFoot"); body_queries.push_back("rightFoot"); body_queries.push_back("pelvis");              

    ihmc_msgs::PelvisTrajectoryRosMessage   pelvis_trajectory_message;
    if (ik_manager.FK_bodies(ik_init_robot_state, body_queries, body_poses)){
        double traj_time = 2.0;
        float des_pelvis_x = (body_poses[0].position.x + body_poses[1].position.x)/2.0;
        float des_pelvis_y = (body_poses[0].position.y + body_poses[1].position.y)/2.0;
        float des_pelvis_z = 1.0;

        geometry_msgs::Vector3 linear_velocity; 
        geometry_msgs::Vector3 angular_velocity; 
        angular_velocity.x = 0.0;
        angular_velocity.y = 0.0;
        angular_velocity.z = 0.0;
        // Prepare Pelvis SE(3) Trajectory Message
            ihmc_msgs::SE3TrajectoryPointRosMessage     start_SE3_pelvis_traj;
            start_SE3_pelvis_traj.time = 0.0;
            start_SE3_pelvis_traj.position.x = body_poses[2].position.x;
            start_SE3_pelvis_traj.position.y = body_poses[2].position.y;
            start_SE3_pelvis_traj.position.z = body_poses[2].position.z;                        
            start_SE3_pelvis_traj.orientation = body_poses[2].orientation;
            start_SE3_pelvis_traj.linear_velocity = linear_velocity;
            start_SE3_pelvis_traj.angular_velocity = angular_velocity;  
            start_SE3_pelvis_traj.unique_id = GO_HOME_ID;       

            ihmc_msgs::SE3TrajectoryPointRosMessage     end_SE3_pelvis_traj;
            end_SE3_pelvis_traj.time = traj_time;
            end_SE3_pelvis_traj.position.x = des_pelvis_x;
            end_SE3_pelvis_traj.position.y = des_pelvis_y;
            end_SE3_pelvis_traj.position.z = des_pelvis_z;                        
            end_SE3_pelvis_traj.orientation = body_poses[2].orientation;
            end_SE3_pelvis_traj.linear_velocity = linear_velocity;
            end_SE3_pelvis_traj.angular_velocity = angular_velocity;  
            end_SE3_pelvis_traj.unique_id = GO_HOME_ID;

        pelvis_trajectory_message.execution_mode = 0;
        pelvis_trajectory_message.previous_message_id = 0;
        pelvis_trajectory_message.unique_id = GO_HOME_ID;
        pelvis_trajectory_message.taskspace_trajectory_points.push_back(start_SE3_pelvis_traj);
        pelvis_trajectory_message.taskspace_trajectory_points.push_back(end_SE3_pelvis_traj);        

        #ifdef ON_REAL_ROBOT
            pelvis_trajectory_message.frame_information.trajectory_reference_frame_id = 83766130;
            pelvis_trajectory_message.frame_information.data_reference_frame_id = 83766130;         
            pelvis_trajectory_message.use_custom_control_frame = false;
        #endif        

        ROS_INFO("SENDING PELVIS TO z = 1.0");
        ihmc_pelvis_traj_pub.publish(pelvis_trajectory_message);
        ros::Duration(0.5).sleep();
    }

    //Publish GO Home Message
    ihmc_go_home_pub.publish(larm_go_home_msg);
    ros::Duration(0.5).sleep();
    ihmc_go_home_pub.publish(rarm_go_home_msg); 
    ros::Duration(0.5).sleep();
    ihmc_go_home_pub.publish(chest_go_home_msg);
//    ros::Duration(0.5).sleep();
//    ihmc_go_home_pub.publish(pelvis_go_home_msg);        

    ROS_INFO("Finished Sending GO Home messages for arms and chest.");
//    ROS_WARN("Once the robot has gone home, the robot will fail to satisfy desired pelvis trajectories");    

}

void LogicManager::try_grasp(int index){
    if (hand_to_use == RIGHT_HAND){
        if ( (right_hand_grasps.size() > 0) && (index < right_hand_grasps.size()) ){
            // Calculate IK
            if (ik_manager.calc_single_hand_IK( right_hand_grasps[index] , RIGHT_HAND, ik_init_robot_state, ik_final_robot_state)){
                ROS_INFO("calc_single_hand_success!");
                publish_ik_final_state_viz();
            }else{
                ROS_WARN("Failed to calculate desired right hand pose IK");
            }

        }else{
            ROS_ERROR("There are no stored right hand grasps. Failed to solve IK for grasp");
        }
    }
    else if(hand_to_use == LEFT_HAND){
        if ( (left_hand_grasps.size() > 0) && (index < left_hand_grasps.size()) ){
        // Calculate IK
            if (ik_manager.calc_single_hand_IK( left_hand_grasps[index] , LEFT_HAND, ik_init_robot_state, ik_final_robot_state)){
                ROS_INFO("calc_single_hand_success!");
                publish_ik_final_state_viz();
            }else{
                ROS_WARN("Failed to calculate desired left hand pose IK");
            }
        }else{
            ROS_ERROR("There are no stored left hand grasps. Failed to solve IK for grasp");
        }
    }

}




void LogicManager::init_sample_object_marker(){
     visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/world";
    //marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "box_grab";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0.6;
    marker.pose.position.y = 0.05;
    marker.pose.position.z = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration();

    // Copy Marker
    sample_object_marker = marker;

}