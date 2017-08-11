#!/usr/bin/env python
import util_quat as quat
import numpy

def get_pelvis_xy_coplanar_quat(q_pelvis_in):
	q_pelvis = quat.Normalize(q_pelvis_in)
	# Convert Quat to R matrix
	R_pelvis = quat.quat_to_R(q_pelvis)

	# Get Columns of R matrix
	pelvis_x_dir = R_pelvis[0][:]
	pelvis_y_dir = R_pelvis[1][:]
	pelvis_z_dir = R_pelvis[2][:]		

	# Set directions to be coplanar to x-y and normalize the directions
	pelvis_x_dir = pelvis_x_dir /  numpy.linalg.norm(pelvis_x_dir)
	pelvis_y_dir = pelvis_y_dir /  numpy.linalg.norm(pelvis_y_dir)
	pelvis_z_dir = numpy.array([0,0,1.0])

	# Create new R matrix representing only the yaw of the pelvis
	R_pelvis_yaw = numpy.array([pelvis_x_dir, pelvis_y_dir, pelvis_z_dir])

	# Get the quaternion of the R matrix
	quat_pelvis_yaw = quat.R_to_quat(R_pelvis_yaw)

	print "Pelvis exp representation: (theta, axis_of_rotation_hat)", quat.quat_to_wth(quat_pelvis_yaw)

	return quat_pelvis_yaw


def raw_test():
	#	q_pelvis = numpy.array([0.92, 0, 0 ,0.375]) # Around 45 degrees
	q_pelvis = numpy.array([0.92387953251, 0, 0 ,0.38268343236]) # Around 45 degrees
	print q_pelvis
	q_pelvis = quat.Normalize(q_pelvis)
	print q_pelvis

	R_pelvis = quat.quat_to_R(q_pelvis)
	print "R_pelvis"
	print R_pelvis

	pelvis_x_dir = R_pelvis[0][:]
	pelvis_y_dir = R_pelvis[1][:]
	pelvis_z_dir = R_pelvis[2][:]		

	print "pelvis_x_dir", pelvis_x_dir
	print "pelvis_y_dir", pelvis_y_dir
	print "pelvis_z_dir", pelvis_z_dir

	print "Normalized and co-planar to x-y plane"
	pelvis_x_dir = pelvis_x_dir /  numpy.linalg.norm(pelvis_x_dir)
	pelvis_y_dir = pelvis_y_dir /  numpy.linalg.norm(pelvis_y_dir)
	pelvis_z_dir = numpy.array([0,0,1.0])

	print "pelvis_x_dir", pelvis_x_dir
	print "pelvis_y_dir", pelvis_y_dir
	print "pelvis_z_dir", pelvis_z_dir

	R_pelvis_yaw = numpy.array([pelvis_x_dir, pelvis_y_dir, pelvis_z_dir])

	print "R_pelvis_yaw"
	print R_pelvis_yaw
	quat_pelvis_yaw = quat.R_to_quat(R_pelvis_yaw)

	print "quat_pelvis_yaw"
	print quat_pelvis_yaw	
	print "(theta, axis_of_rotation_hat)", quat.quat_to_wth(quat_pelvis_yaw)

if __name__ == '__main__':
	print " "
	print "Begin Test"
	raw_test()
	print "Begin Function Test"

	q_pelvis_world = numpy.array([0.92387953251, 0, 0 ,0.38268343236]) # Around 45 degrees
	get_pelvis_xy_coplanar_quat(q_pelvis_world)


# get pelvis quat

# quat to R

# get pelvis_x_dir
# get pelvis_y_dir
# set their z's to 0
# normalize pelvis_x_dir
# normalize pelvis_y_dir
# set pelvis_z_dir = 1.0

# Construct R matrix
# R -> quat


# Use this orientation for stepping forwards and backwards

# Pelvis at +45 Degrees
# x = 0, y = 0, z = 0.375, w = 0.9283

# left_foot_world = self.tfBuffer.lookup_transform(
#     'world', self.LEFT_FOOT_FRAME_NAME, rospy.Time())

