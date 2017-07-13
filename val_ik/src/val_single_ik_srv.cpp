#include "val_ik/val_single_ik_srv.h"
void Single_IK_srv::get_pelvis_RPY(val_ik_msgs::RobotState &robot_state, double &pelvis_roll, double &pelvis_pitch, double &pelvis_yaw){
	// Set Matrix from Quaternion Information
	tf::Quaternion q(robot_state.robot_pose.orientation.x, robot_state.robot_pose.orientation.y,	
					 robot_state.robot_pose.orientation.z, robot_state.robot_pose.orientation.w);
	tf::Matrix3x3  rotMatrix;
	rotMatrix.setRotation(q);
	// Get Roll Pitch Yaw:
	rotMatrix.getRPY(pelvis_roll, pelvis_pitch, pelvis_yaw);
}


// This function fills in the robot state names and value of ik_srv from the given robot_state
void Single_IK_srv::init_drake_states(val_ik_msgs::RobotState &robot_state, val_ik::DrakeIKVal &ik_srv){
/*	geometry_msgs::Pose robot_state.robot_pose
	sensor_msgs::JointState robot_state.body_joint_states*/	

	// Initialize state names and their value
	for(int i = 0; i < val_ik_global::drake_robot_state_names.size() ; i++){
		std::string state_name;
		state_name = val_ik_global::drake_robot_state_names[i];
		ik_srv.request.drake_robot_state_names.push_back( state_name ); // Fill in drake_robot_state_names;
		ik_srv.request.init_drake_robot_states_value.push_back( 0.0 );	// enter 0.0 for the init_drake_robot_state_value;	
	}

	// Fill in ik_srv joint state values from robot_state  
	for(size_t i = 0; i < robot_state.body_joint_states.name.size(); i++){
		std::string joint_name; 
		joint_name = robot_state.body_joint_states.name[i];
 		float joint_value = robot_state.body_joint_states.position[i];

		std::map<std::string, int>::const_iterator iter = val_ik_global::drake_state_name_to_state_index.find(joint_name);
		if(iter != val_ik_global::drake_state_name_to_state_index.end()){
		    // Item in the map. The value will be accessible as `iter->second`.
	   		std::cout << "  Found robot state " << joint_name << " in the map" << std::endl;			
			int state_index = val_ik_global::drake_state_name_to_state_index.at(joint_name);
			ik_srv.request.init_drake_robot_states_value[state_index] = joint_value;
		}else{
	   		std::cout << "  Did not find robot state " << joint_name << " in the map" << std::endl;			
		}

	}

	// Fill in Pelvis X,Y,Z
	ik_srv.request.init_drake_robot_states_value[0] = robot_state.robot_pose.position.x;
	ik_srv.request.init_drake_robot_states_value[1] = robot_state.robot_pose.position.y;
	ik_srv.request.init_drake_robot_states_value[2] = robot_state.robot_pose.position.z;		


	// Fill in Pelvis Roll Pitch Yaw
	double pelvis_roll_val; double pelvis_pitch_val; double pelvis_yaw_val;
	get_pelvis_RPY(robot_state, pelvis_roll_val, pelvis_pitch_val, pelvis_yaw_val);

	// Set ik_srv roll pitch yaw
	ik_srv.request.init_drake_robot_states_value[3] = pelvis_roll_val;
	ik_srv.request.init_drake_robot_states_value[4] = pelvis_pitch_val;
	ik_srv.request.init_drake_robot_states_value[5] = pelvis_yaw_val;


	// DEBUG OUTPUTS
	// IK srv debug:
	std::cout << "Begin debug ik_srv" << std::endl;
	for (size_t i = 0; i < val_ik_global::drake_robot_state_names.size(); i++){
		std::cout << ik_srv.request.drake_robot_state_names[i] << " " << ik_srv.request.init_drake_robot_states_value[i] << std::endl;
	}

	// Debug input robot joint names and position:
	std::cout << "Begin debug input" << std::endl;
	for(size_t i = 0; i < robot_state.body_joint_states.name.size(); i++){
		std::string joint_name; 
		joint_name = robot_state.body_joint_states.name[i];
		std::cout << joint_name << " " << robot_state.body_joint_states.position[i] << std::endl;
	}

}

void Single_IK_srv::define_desired_hand_pos(val_ik::DrakeOneHandSingleIk::Request& req, val_ik::DrakeIKVal &ik_srv){
    val_ik_msgs::BodyPositionConstraint body_constraint;
    val_ik_msgs::BodyQuaternionConstraint body_quat_constraint;    

    body_constraint.body_name = "rightPalm";    
    body_constraint.offset_from_current = false;
    body_constraint.world_position.x = req.des_hand_pose.position.x;
    body_constraint.world_position.y = req.des_hand_pose.position.y;
    body_constraint.world_position.z = req.des_hand_pose.position.z;

	body_quat_constraint.body_name = "rightPalm";    
    body_quat_constraint.offset_from_current = false;
    body_quat_constraint.quaternion = req.des_hand_pose.orientation;

    ik_srv.request.desired_body_positions.push_back(body_constraint);
    ik_srv.request.desired_quaternion_positions.push_back(body_quat_constraint);
}


bool Single_IK_srv::OneHandSingleIk_callback(val_ik::DrakeOneHandSingleIk::Request& req, val_ik::DrakeOneHandSingleIk::Response& res){
	val_ik::DrakeIKVal ik_srv;
	init_drake_states(req.robot_state, ik_srv);
	define_desired_hand_pos(req, ik_srv);

    // Begin Service Call
    if (val_ik_client.call(ik_srv)){
        ROS_INFO("val_ik/val_ik_service Call Successful");
        // Returning Message
        val_ik_msgs::RobotState robot_state_msg_res;
        convert_ik_srv_res_to_robot_state(ik_srv, robot_state_msg_res);
        res.robot_state = robot_state_msg_res;
    }
    else{
       ROS_ERROR("Failed to call service val_ik/val_ik_service");
       return false;
    }
    

    ROS_INFO("Single IK Call Successful");
	return true;
}

void Single_IK_srv::convert_ik_srv_res_to_robot_state(val_ik::DrakeIKVal &ik_srv, val_ik_msgs::RobotState &robot_state_msg){
    tf::Quaternion q;
    float body_x = ik_srv.response.robot_joint_states.floating_joint_states.position[0]; 
    float body_y = ik_srv.response.robot_joint_states.floating_joint_states.position[1];
    float body_z = ik_srv.response.robot_joint_states.floating_joint_states.position[2];    

    float body_roll = ik_srv.response.robot_joint_states.floating_joint_states.position[3]; 
    float body_pitch = ik_srv.response.robot_joint_states.floating_joint_states.position[4];
    float body_yaw = ik_srv.response.robot_joint_states.floating_joint_states.position[5]; 

    // Rotation from fixed axis
    tf::Quaternion q_world_roll;   q_world_roll.setRPY(body_roll, 0.0, 0.0); 
    tf::Quaternion q_world_pitch;  q_world_pitch.setRPY(0.0, body_pitch, 0.0); 
    tf::Quaternion q_world_yaw;    q_world_yaw.setRPY(0.0, 0.0, body_yaw); 

    // Normalize Quaternions
    q_world_roll.normalize();     q_world_pitch.normalize();     q_world_yaw.normalize();

    // Extrinsict Rotation about Space-fixed x-y-z axes by R-P-Y angles respectively.
    // equivalently, we use an instrinsic rotation about body frames z-y`-x`` 
    // Convention of Drake as seen in math::rol_pitch_yaw_not_using_quaternions
    // q = q_world_yaw*q_world_pitch*q_world_roll;
    q.setRPY(body_roll , body_pitch , body_yaw);

    // Prepare robot_pose
    geometry_msgs::Pose robot_pose;
    robot_pose.position.x = body_x;     robot_pose.position.y = body_y;     robot_pose.position.z = body_z;
	tf::quaternionTFToMsg 	(q, robot_pose.orientation); 		

	// Set Message
    robot_state_msg.robot_pose = robot_pose;
    robot_state_msg.body_joint_states = ik_srv.response.robot_joint_states.body_joint_states;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "val_ik_single_ik_server_node");

    Single_IK_srv single_ik_srv_obj;

    // Declare Client Dependency
    single_ik_srv_obj.val_ik_client = single_ik_srv_obj.nh.serviceClient<val_ik::DrakeIKVal>("val_ik/val_ik_service");

    // See http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
    // for additional arguments http://answers.ros.org/question/46784/pass-additional-arguments-into-advertiseservice/
	single_ik_srv_obj.val_ik_single_srv = single_ik_srv_obj.nh.advertiseService("val_ik/val_single_ik_service", &Single_IK_srv::OneHandSingleIk_callback, 
																												&single_ik_srv_obj);
    ROS_INFO("Waiting for val_ik_service");
    ros::service::waitForService("val_ik/val_ik_service", -1);
    ROS_INFO("Service is ready");

//    val_ik_global::output_global_vars();

    ros::spin();
	return 0;
}