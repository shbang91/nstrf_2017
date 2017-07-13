#include "robot_state.h"
RobotState::RobotState(){
    valid_fields = false;
}
RobotState::~RobotState(){}

void RobotState::getRPY(double &pelvis_roll, double &pelvis_pitch, double &pelvis_yaw){
	// Set Matrix from Quaternion Information
	tf::Quaternion q(robot_pose.pose.pose.orientation.x, robot_pose.pose.pose.orientation.y,	
					 robot_pose.pose.pose.orientation.z, robot_pose.pose.pose.orientation.w);
	tf::Matrix3x3  rotMatrix;
	rotMatrix.setRotation(q);

	// Get Roll Pitch Yaw:
	rotMatrix.getRPY(pelvis_roll, pelvis_pitch, pelvis_yaw);
}

void RobotState::getXYZ(double &pelvis_x, double &pelvis_y, double &pelvis_z){
	pelvis_x = robot_pose.pose.pose.position.x;
	pelvis_y = robot_pose.pose.pose.position.y;
	pelvis_z = robot_pose.pose.pose.position.z;	
}

void RobotState::publish_viz(ros::Publisher &state_publisher,  tf::TransformBroadcaster &br){
    tf::Transform transform;
/*	tf::Quaternion q(robot_pose.pose.pose.orientation.x, robot_pose.pose.pose.orientation.y,	
				     robot_pose.pose.pose.orientation.z, robot_pose.pose.pose.orientation.w);
    transform.setRotation(q);
*/

    double pelvis_x; double pelvis_y; double pelvis_z;
    getXYZ(pelvis_x, pelvis_y, pelvis_z);

    tf::Quaternion q;
    double pelvis_roll; double pelvis_pitch; double pelvis_yaw;
    getRPY(pelvis_roll, pelvis_pitch, pelvis_yaw);     

    // Find Quaternion Equivalent of each fixed-axis rotation 
    tf::Quaternion q_world_roll;   q_world_roll.setRPY(pelvis_roll, 0.0, 0.0); 
    tf::Quaternion q_world_pitch;  q_world_pitch.setRPY(0.0, pelvis_pitch, 0.0); 
    tf::Quaternion q_world_yaw;    q_world_yaw.setRPY(0.0, 0.0, pelvis_yaw); 
    
    // Find Quaternion Equivalent of RPY
    q.setRPY(pelvis_roll , pelvis_pitch , pelvis_yaw);

    transform.setOrigin( tf::Vector3(pelvis_x, pelvis_y, pelvis_z) );
    transform.setRotation(q);

    std::string frame_id;
    frame_id = robot_namespace + "/pelvis";
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", frame_id));
    
    state_publisher.publish(joint_state);
    
}