#include "logic_main.h"
LogicManager::LogicManager(){
	init_sample_object_marker();
    ik_init_robot_state.robot_namespace = "val_ik_initpose_robot";
    ik_final_robot_state.robot_namespace = "val_ik_finalpose_robot";
} // Constructor
LogicManager::~LogicManager(){} // Destructor



void LogicManager::publish_ik_init_state_viz(){
    ik_init_robot_state.publish_viz(val_ik_initpose_robot_joint_states_pub, br);    
}


void LogicManager::update_current_robot_state(){
    if (global_state_update_received){
        current_robot_state.joint_state = global_joint_state_msg;
        current_robot_state.robot_pose = global_odom_msg; 

        // Test
        ik_init_robot_state.joint_state = current_robot_state.joint_state;
        ik_init_robot_state.robot_pose = current_robot_state.robot_pose;
        publish_ik_init_state_viz();
        // Test


        ROS_INFO("State Update Received");
        ROS_INFO("JointState Size = %zu", global_joint_state_msg.position.size());
        global_state_update_received = false;
    }
}

void LogicManager::loop(){
    update_current_robot_state();
    sample_object_marker.header.stamp = ros::Time::now();
    marker_pub.publish(sample_object_marker);

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

    // set initial IK position from current Robot Position
    
    // Prepare msg for IK service
        // call Forward Kinematics to get current left and right foot positions 

    // Store left and right foot positions
    
    // prepare IK msg constraints

    // set initial IK constraints
    // constrain foot positions
    // constrain robot_side hand position and orientation




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