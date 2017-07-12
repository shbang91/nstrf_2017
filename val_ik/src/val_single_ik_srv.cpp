#include "ros/ros.h"
#include "val_ik/global_vars.h"
#include "val_ik/DrakeOneHandSingleIk.h"
#include "val_ik/DrakeIKVal.h"

#include "val_ik_msgs/RobotState.h"
/* Input:
uint8				   robot_side
val_ik_msgs/RobotState robot_state
geometry_msgs/Pose 	   des_hand_pose

Output
val_ik_msgs/RobotState robot_state
*/


class Single_IK_srv{
public:
	ros::NodeHandle 			nh;
	ros::ServiceServer      	val_ik_single_srv;
	ros::ServiceClient			val_ik_client;
 
	// convert val_ik_msgs/RobotState to
	//   string[] global_drake_floating_joint_names
	//   string[] global_drake_robot_state_names
	//   float32[] init_drake_body_joint_pos
	//   float32[] init_drake_floating_joint_pos
	void quat_to_RPY();

	void init_drake_states(val_ik_msgs::RobotState &robot_state,  val_ik::DrakeIKVal &ik_srv);
	void callDrake_IK();
	bool OneHandSingleIk_callback(val_ik::DrakeOneHandSingleIk::Request& req, val_ik::DrakeOneHandSingleIk::Response& res);	
};

/*
    // Prepare Client Message
    std::vector<std::string>                                 drake_floating_joint_names;
    std::vector<std::string>                                 drake_body_joint_names;
    std::vector<float>                                       init_drake_body_joint_pos;
    std::vector<float>                                       init_drake_floating_joint_pos;
    // string[]  drake_robot_state_names
	// float32[] init_drake_robot_states
    std::vector<val_ik_msgs::BodyPositionConstraint>         desired_body_positions;
    std::vector<val_ik_msgs::BodyQuaternionConstraint>       desired_quaternion_positions;
    std::vector<val_ik_msgs::JointPositionConstraint>        desired_joint_positions;

    // Initialize IK Joint Names
    init_IK_joint_names(drake_floating_joint_names, drake_body_joint_names);
    init_IK_positions(drake_floating_joint_names, drake_body_joint_names, init_drake_floating_joint_pos, init_drake_body_joint_pos);
    define_IK_init_positions_test(init_drake_floating_joint_pos, init_drake_body_joint_pos);
    define_desired_hand_pos(desired_body_positions, 0.0, 0.0, 0.0, true);    

    ik_srv.request.drake_floating_joint_names = drake_floating_joint_names;
    ik_srv.request.drake_body_joint_names = drake_body_joint_names;    

    ik_srv.request.init_drake_body_joint_pos = init_drake_body_joint_pos;
    ik_srv.request.init_drake_floating_joint_pos = init_drake_floating_joint_pos;   

    ik_srv.request.desired_body_positions = desired_body_positions;
    ik_srv.request.desired_quaternion_positions = desired_quaternion_positions;        
    ik_srv.request.desired_joint_positions = desired_joint_positions;    
*/



void Single_IK_srv::init_drake_states(val_ik_msgs::RobotState &robot_state, val_ik::DrakeIKVal &ik_srv){

/*	geometry_msgs::Pose robot_state.robot_pose
	sensor_msgs::JointState robot_state.body_joint_states*/	
	for(int i = 0; i < val_ik_global::drake_robot_state_names.size() ; i++){
		std::string state_name;
		state_name = val_ik_global::drake_robot_state_names[i];
		ik_srv.request.drake_robot_state_names.push_back( state_name ); // Fill in drake_robot_state_names;
		ik_srv.request.init_drake_robot_states_value.push_back( 0.0 );	// enter 0.0 for the init_drake_robot_state_value;	
	}

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


bool Single_IK_srv::OneHandSingleIk_callback(val_ik::DrakeOneHandSingleIk::Request& req, val_ik::DrakeOneHandSingleIk::Response& res){
	val_ik::DrakeIKVal ik_srv;
	init_drake_states(req.robot_state, ik_srv);

	return true;
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

/*	val_ik_msgs::RobotState robot_state;
	robot_state.body_joint_states.name.push_back("leftHipRoll");
	robot_state.body_joint_states.position.push_back(0.5);
	robot_state.body_joint_states.name.push_back("randomJoint");
	robot_state.body_joint_states.position.push_back(1.0);

	val_ik::DrakeIKVal      ik_srv;
	single_ik_srv_obj.init_drake_states(robot_state, ik_srv);*/

    ROS_INFO("Waiting for val_ik_service");
    ros::service::waitForService("val_ik/val_ik_service", -1);
    ROS_INFO("Service is ready");

//    val_ik_global::output_global_vars();

    ros::spin();
	return 0;
}