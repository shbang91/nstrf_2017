#include "logic_main.h"

void  LogicManager::interactive_callback(const visualization_msgs::InteractiveMarkerInitConstPtr& msg){
	ROS_INFO("Logic Manager Interactive Marker Callback received");	

	if (msg->markers.size() > 0){
		geometry_msgs::Pose 	   marker_pose;
		marker_pose = msg->markers[0].pose;

		// Publish a Transform for Visualization
		tf::Transform transform;
		tf::Quaternion q(msg->markers[0].pose.orientation.x, msg->markers[0].pose.orientation.y, msg->markers[0].pose.orientation.z, msg->markers[0].pose.orientation.w );
		transform.setOrigin( tf::Vector3(msg->markers[0].pose.position.x, msg->markers[0].pose.position.y, msg->markers[0].pose.position.z) );
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "basic_controls/im_frame"));


		// Check that we have received at least a single initialized ik 
		if	(global_first_state_update_received and ik_init_robot_state.valid_fields){
			// Prepare initial IK robot state
		    ik_init_robot_state.joint_state = current_robot_state.joint_state;
		    ik_init_robot_state.robot_pose = current_robot_state.robot_pose;

		    // Calculate IK
			if (ik_manager.calc_single_hand_IK(marker_pose, 1, ik_init_robot_state, ik_final_robot_state)){
				ROS_INFO("calc_single_hand_success!");
				publish_ik_final_state_viz();
			}else{
				ROS_WARN("Failed to calculate desired hand pose IK");
			}

		}

	}

}

void  LogicManager::operator_command_callback(const std_msgs::StringConstPtr& msg){
	ROS_INFO("Logic Manager Operator Command Callback");	
	std::string send_single_ik_wbc;		 send_single_ik_wbc = "send_single_ik";
	std::string testFK;					 testFK = "testFK";
	std::string go_home;				 go_home = "go_home";
	std::string re_init_markers;		 re_init_markers = "re_init_markers";
	std::string run_grasploc;            run_grasploc = "run_grasploc";
	std::string get_nearest_grasp_ik;    get_nearest_grasp_ik = "get_nearest_grasp_ik";
	std::string try_next_grasp_ik;       try_next_grasp_ik = "try_next_grasp_ik";

	if (send_single_ik_wbc.compare(msg->data) == 0){
		ROS_INFO("Attempting to send single IK WBC");
		sendSingleIKWBC();
	}else if(testFK.compare(msg->data) == 0){
		ROS_INFO("Attempting to call FK");
		std::vector<std::string> body_queries;
		std::vector<geometry_msgs::Pose> body_poses;

		body_queries.push_back("torso"); body_queries.push_back("rightPalm");				
		ik_manager.FK_bodies(ik_init_robot_state, body_queries, body_poses);       

	}else if(go_home.compare(msg->data) == 0){
		ROS_INFO("Sending Robot Home (to neutral position for walking)");
		sendWBCGoHome();
	}else if(re_init_markers.compare(msg->data) == 0){
		ROS_INFO("Interactive Markers are being Reset. IM server will handle it");
	}else if(run_grasploc.compare(msg->data) == 0){
		ROS_INFO("Calling Grasploc. Grasploc server will handle it");
	}else if(get_nearest_grasp_ik.compare(msg->data) == 0){
		ROS_INFO("Finding IK For nearest Grasp");
		try_grasp(0);
	}else if(try_next_grasp_ik.compare(msg->data) == 0){
		ROS_INFO("Find IK for next Grasp");
		if (right_hand_graps.size() > 0){
		    righthand_grasp_index = (righthand_grasp_index + 1) % right_hand_graps.size();
			try_grasp(righthand_grasp_index);
		}else{
			ROS_ERROR("There are no stored grasps.");
		}
	}


	else{
		ROS_WARN("Unknown Operator Command");
	}


}

void LogicManager::grasploc_callback(const valkyrie::GraspHandPosesConstPtr& msg){
	ROS_INFO("Received Grasploc Callback. Storing New Grasps");
	right_hand_graps.clear();

	std::cout << "Number of grasps:" << msg->right_hand_pose.size() << std::endl;

	for(size_t i = 0; i < (msg->right_hand_pose.size()); i++){
		//if (msg->right_z_up[i] == 1){
			right_hand_graps.push_back( msg->right_hand_pose[i]);
		//}
	}

	ROS_INFO("Stored: %zu right hand grasps", right_hand_graps.size());


} //grasploc

void stateFiltersCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg, const nav_msgs::OdometryConstPtr& odom_msg){
	state_mutex.lock();
		global_joint_state_msg = (*joint_state_msg);
		global_odom_msg = (*odom_msg);
		global_first_state_update_received = true;
		global_state_update_received = true;		
	state_mutex.unlock();
}