#include "ros/ros.h"
#include "val_ik/global_vars.h"
#include "val_ik/DrakeOneHandSingleIk.h"
#include "val_ik/DrakeIKVal.h"

#include "val_ik_msgs/RobotState.h"
#include "val_ik_msgs/BodyPositionConstraint.h"
#include <tf/tf.h>

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
	void init_drake_states(val_ik_msgs::RobotState &robot_state,  val_ik::DrakeIKVal &ik_srv);
	void define_desired_hand_pos(val_ik::DrakeOneHandSingleIk::Request& req, val_ik::DrakeIKVal &ik_srv);

	void get_pelvis_RPY(val_ik_msgs::RobotState &robot_state, double &pelvis_roll, double &pelvis_pitch, double &pelvis_yaw);

	bool OneHandSingleIk_callback(val_ik::DrakeOneHandSingleIk::Request& req, val_ik::DrakeOneHandSingleIk::Response& res);	
};
