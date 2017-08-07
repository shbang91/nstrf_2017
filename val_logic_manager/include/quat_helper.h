#ifndef QUAT_HELP_H
#define QUAT_HELP_H

#include <ros/ros.h>
#include <tf/tf.h>
#include "geometry_msgs/Pose.h"

#include <math.h>
#include <Eigen/Dense>


typedef Eigen::Matrix<float, 3, 3> RotMat3f;
typedef Eigen::Matrix<float, 3, 1> Vector3f;

Vector3f get_RotMat_col(RotMat3f R, int index);
RotMat3f quat_to_R(geometry_msgs::Quaternion quat_msg);
geometry_msgs::Quaternion R_to_quat(RotMat3f R);


#endif