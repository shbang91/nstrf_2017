#include "logic_main.h"

void  LogicManager::interactive_callback(const visualization_msgs::InteractiveMarkerInitConstPtr& msg){
	ROS_INFO("Logic Manager Interactive Marker Callback received");	

	visualization_msgs::InteractiveMarkerInit im_msg;
	im_msg = (*msg);

	if (msg->markers.size() > 0){

		geometry_msgs::Pose 	   marker_pose;
		marker_pose = msg->markers[0].pose;

		std::cout << "marker quat: " <<
				 msg->markers[0].pose.orientation.x << " " <<
				 msg->markers[0].pose.orientation.y << " " <<
				 msg->markers[0].pose.orientation.z << " " <<
				 msg->markers[0].pose.orientation.w << std::endl;


		tf::Transform transform;
		tf::Quaternion q(msg->markers[0].pose.orientation.x, msg->markers[0].pose.orientation.y, msg->markers[0].pose.orientation.z, msg->markers[0].pose.orientation.w );
		transform.setOrigin( tf::Vector3(msg->markers[0].pose.position.x, msg->markers[0].pose.position.y, msg->markers[0].pose.position.z) );
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "basic_controls/im_frame"));


		if	(global_first_state_update_received and ik_init_robot_state.valid_fields){
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
			single_ik_srv.request.des_hand_pose = marker_pose;

			if (single_ik_client.call(single_ik_srv)){
		        ROS_INFO("    Single IK Call Successful");
		        // Store to final IK position
		        ik_final_robot_state.robot_pose.pose.pose = single_ik_srv.response.robot_state.robot_pose;
		        ik_final_robot_state.joint_state = single_ik_srv.response.robot_state.body_joint_states;
		        ik_final_robot_state.valid_fields = true;	
		        publish_ik_final_state_viz();
		    }
		    else{
		       ROS_WARN("    Failed to call Single IK service");
		    }
		}
	}

}

void  LogicManager::operator_command_callback(const std_msgs::StringConstPtr& msg){
	ROS_INFO("Logic Manager Operator Command Callback");	
	std::string send_single_ik_wbc;
	send_single_ik_wbc = "send_single_ik";

	std::string testFK;
	testFK = "testFK";
	if (send_single_ik_wbc.compare(msg->data) == 0){
		ROS_INFO("Attempting to send single IK WBC");
		sendSingleIKWBC();
	}else if(testFK.compare(msg->data) == 0){
		ROS_INFO("Attempting to call FK");
		std::vector<std::string> body_queries;
		std::vector<geometry_msgs::Pose> body_poses;

		body_queries.push_back("torso");
		body_queries.push_back("rightPalm");				
		ik_manager.FK_bodies(fk_client,	ik_init_robot_state, body_queries, body_poses);       

 		for (size_t i = 0; i < body_poses.size(); i ++ ){
 			std::cout << "Body:" << body_queries[i] << std::endl;
        	std::cout << "    pos x:" << body_poses[i].position.x << std::endl;
        	std::cout << "    pos y:" << body_poses[i].position.y << std::endl;
        	std::cout << "    pos z:" << body_poses[i].position.z << std::endl;
        	std::cout << "    quat x:" << body_poses[i].orientation.x << std::endl;
        	std::cout << "    quat y:" << body_poses[i].orientation.y << std::endl;
        	std::cout << "    quat z:" << body_poses[i].orientation.z << std::endl;
        	std::cout << "    quat w:" << body_poses[i].orientation.w << std::endl;        	        	        	
        }

	}
	else{
		ROS_WARN("Unknown Operator Command");
	}




}

void stateFiltersCallback(const sensor_msgs::JointStateConstPtr& joint_state_msg, const nav_msgs::OdometryConstPtr& odom_msg){
	state_mutex.lock();
		global_joint_state_msg = (*joint_state_msg);
		global_odom_msg = (*odom_msg);
		global_first_state_update_received = true;
		global_state_update_received = true;		
	state_mutex.unlock();
}