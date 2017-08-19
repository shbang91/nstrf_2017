#include "quat_helper.h"
Vector3f get_RotMat_col(RotMat3f R, int index){
	Vector3f col(R(0, index), R(1, index), R(2, index) );
	return col;
}


RotMat3f quat_to_R(geometry_msgs::Quaternion quat_msg){
	tf::Quaternion tf_quat;
	tf::quaternionMsgToTF(quat_msg, tf_quat);
	tf_quat.normalize();

	geometry_msgs::Quaternion quat; 
	tf::quaternionTFToMsg 	(tf_quat, quat); 	

	float q0 = quat.w;
	float q1 = quat.x;
	float q2 = quat.y;
	float q3 = quat.z;

	float r11 = pow(q0, 2) + pow(q1, 2) - pow(q2, 2) - pow(q3, 2);
	float r21 = 2.0*(q0*q3 + q1*q2);
	float r31 = 2.0*(q1*q3 - q0*q2);

	float r12 = 2.0*(q1*q2 - q0*q3);
	float r22 = pow(q0, 2) - pow(q1, 2) + pow(q2, 2) - pow(q3, 2);
	float r32 = 2.0*(q0*q1 + q2*q3);

	float r13 = 2.0*(q0*q2 + q1*q3);
	float r23 = 2.0*(q2*q3 - q0*q1);
	float r33 = pow(q0, 2) - pow(q1, 2) - pow(q2, 2) + pow(q3, 2); 	


	RotMat3f R;
	R(0,0) = r11; R(0,1) = r12; R(0,2) = r13; 
	R(1,0) = r21; R(1,1) = r22; R(1,2) = r23;
	R(2,0) = r31; R(2,1) = r32; R(2,2) = r33;

	return R;				
}

geometry_msgs::Quaternion R_to_quat(RotMat3f R){
	float q0 = 0.5*sqrt(1 + R(0,0) + R(1,1) + R(2,2) );
	float q1 = (1/4.0)*(1/q0)*(R(2,1) - R(1,2));
	float q2 = (1/4.0)*(1/q0)*(R(0,2) - R(2,0));
	float q3 = (1/4.0)*(1/q0)*(R(1,0) - R(0,1));

	geometry_msgs::Quaternion quat_msg;
	quat_msg.w = q0;
	quat_msg.x = q1;
	quat_msg.y = q2;
	quat_msg.z = q3;	

	tf::Quaternion tf_quat;
	tf::quaternionMsgToTF(quat_msg, tf_quat);
	tf_quat.normalize();

	geometry_msgs::Quaternion quat; 
	tf::quaternionTFToMsg 	(tf_quat, quat); 

	return quat;		
}

