#include "logic_main.h"
LogicManager::LogicManager(){
	init_sample_object_marker();
    ik_init_robot_state.robot_namespace = "val_ik_initpose_robot";
    ik_final_robot_state.robot_namespace = "val_ik_finalpose_robot";
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

    //Publish GO Home Message
    ihmc_go_home_pub.publish(larm_go_home_msg);
    ros::Duration(0.5).sleep();
    ihmc_go_home_pub.publish(rarm_go_home_msg); 
    ros::Duration(0.5).sleep();
    ihmc_go_home_pub.publish(chest_go_home_msg);
    ros::Duration(0.5).sleep();
    ihmc_go_home_pub.publish(pelvis_go_home_msg);        

}

// interactiveMarker callback();
//      calculateIK at Marker Position and Orientation
//      
// Send WB Reach Trajectory to IHMC
// getGrabPose
// calculateIK at SE(3);
// 

/*
void calcIK(geometry_msgs::Pose des_hand_pose, int robot_side)
    prepare initial IK pose

    ros service call singleIK




*/

//void LogicManager::publish_ik_final_state_viz(){}



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