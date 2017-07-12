#include "ros/ros.h"
#include "val_ik/global_vars.h"
#include "val_ik/DrakeTwoHandsSingleIk.h"
#include "val_ik/DrakeIKVal.h"

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
 
    val_ik::DrakeIKVal ik_srv;
	// convert val_ik_msgs/RobotState to
	//   string[] global_drake_floating_joint_names
	//   string[] global_drake_robot_state_names
	//   float32[] init_drake_body_joint_pos
	//   float32[] init_drake_floating_joint_pos
	void init_drake_states();
	void callDrake_IK();
	bool OneHandSingleIk_callback(val_ik::DrakeTwoHandsSingleIk::Request& req, val_ik::DrakeTwoHandsSingleIk::Response& res);	
};

bool Single_IK_srv::OneHandSingleIk_callback(val_ik::DrakeTwoHandsSingleIk::Request& req, val_ik::DrakeTwoHandsSingleIk::Response& res){
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

    ROS_INFO("Waiting for val_ik_service");
    ros::service::waitForService("val_ik/val_ik_service", -1);
    ROS_INFO("Service is ready");

    val_ik_global::output_global_vars();

    ros::spin();
	return 0;
}