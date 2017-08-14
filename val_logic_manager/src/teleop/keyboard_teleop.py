#!/usr/bin/env python

from collections import OrderedDict
from math import cos, radians, sin, sqrt
import select
import sys
import termios
import tty

import util_quat as quat

from geometry_msgs.msg import Quaternion, Transform, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from ihmc_msgs.msg import AbortWalkingRosMessage
from ihmc_msgs.msg import ArmTrajectoryRosMessage
from ihmc_msgs.msg import FootstepDataListRosMessage
from ihmc_msgs.msg import FootstepDataRosMessage
from ihmc_msgs.msg import FootstepStatusRosMessage
from ihmc_msgs.msg import HeadTrajectoryRosMessage
from ihmc_msgs.msg import NeckTrajectoryRosMessage
from ihmc_msgs.msg import OneDoFJointTrajectoryRosMessage
from ihmc_msgs.msg import SO3TrajectoryPointRosMessage
from ihmc_msgs.msg import TrajectoryPoint1DRosMessage

import numpy

import rospy

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
import tf2_ros

import argparse



ON_REAL_ROBOT_USE = False
EXECUTE_FOOTSTEPS = True

MESH_LOCATION = "package://val_desc/model/meshes/legs/foot_green.dae"
#ROBOT_MESH_LOCATION = "package://val_desc/model/meshes/legs/foot_green.dae"

# SOLE_FRAME_X_OFFSET = 0.052
# SOLE_FRAME_Y_OFFSET = 0.0
SOLE_FRAME_Z_OFFSET = -0.09


class KeyboardTeleop(object):

    # constants used for walking
    PELVIS_FRAME_NAME = "pelvis"
    LEFT_FOOT_FRAME_NAME = None
    RIGHT_FOOT_FRAME_NAME = None

    if (ON_REAL_ROBOT_USE):
        LEFT_FOOT_FRAME_NAME = "leftFootSoleFrame" #None
        RIGHT_FOOT_FRAME_NAME = "rightFootSoleFrame" #None
    else:
        LEFT_FOOT_FRAME_NAME = "leftFoot" #None
        RIGHT_FOOT_FRAME_NAME = "rightFoot" #None        


    TRANS_STEP = 0.2  # each step will be 20cm
    ROT_STEP = radians(45)  # each rotation will be 45degrees

    def receivedFootStepStatus_cb(self, msg):
        if msg.status == FootstepStatusRosMessage.COMPLETED:
            self.footstep_count += 1

    ARM_BINDINGS = OrderedDict([
        ('q', {'joint_index': 0, 'side': 'left', 'min': -2.85, 'max': 2.0}),  # leftShoulderPitch
        ('w', {'joint_index': 1, 'side': 'left', 'min': -1.519, 'max': 1.266,
               'invert': True}),  # leftShoulderRoll
        ('e', {'joint_index': 2, 'side': 'left', 'min': -3.1, 'max': 2.18}),  # leftShoulderYaw
        ('r', {'joint_index': 3, 'side': 'left', 'min': -2.174, 'max': 0.12,
               'invert': True}),  # leftElbowPitch
        ('a', {'joint_index': 4, 'side': 'left', 'min': -2.019, 'max': 3.14}),  # leftForearmYaw
        ('s', {'joint_index': 5, 'side': 'left', 'min': -0.62, 'max': 0.625,
               'invert': True}),  # leftWristRoll
        ('d', {'joint_index': 6, 'side': 'left', 'min': -0.36, 'max': 0.49,
               'invert': True}),  # leftWristPitch
        ('t', {'joint_index': 'reset', 'side': 'left'}),

        ('u', {'joint_index': 0, 'side': 'right', 'min': -2.85, 'max': 2.0}),  # rightShoulderPitch
        ('i', {'joint_index': 1, 'side': 'right', 'min': -1.266, 'max': 1.519}),  # rightShoulderRoll
        ('o', {'joint_index': 2, 'side': 'right', 'min': -3.1, 'max': 2.18}),  # rightShoulderYaw
        ('p', {'joint_index': 3, 'side': 'right', 'min': -0.12, 'max': 2.174}),  # rightElbowPitch
        ('j', {'joint_index': 4, 'side': 'right', 'min': -2.019, 'max': 3.14}),  # rightForearmYaw
        ('k', {'joint_index': 5, 'side': 'right', 'min': -0.625, 'max': 0.62}),  # rightWristRoll
        ('l', {'joint_index': 6, 'side': 'right', 'min': -0.49, 'max': 0.36}),  # rightWristPitch
        ('y', {'joint_index': 'reset', 'side': 'right'}),
    ])

    HAND_BINDINGS = OrderedDict([
        ('5', {'joint_index': 0, 'side': 'left', 'min': 0.0, 'max': 1.8,
               'uppercase': '%'}),  # left thumb roll
        ('4', {'joint_index': 1, 'side': 'left', 'min': -0.55, 'max': 0.0,
               'uppercase': '$', 'invert': True}),  # left thumb pitch
        ('3', {'joint_index': 2, 'side': 'left', 'min': -1.1, 'max': 0.0,
               'uppercase': '#', 'invert': True}),  # left index
        ('2', {'joint_index': 3, 'side': 'left', 'min': -0.9, 'max': 0.0,
               'uppercase': '@', 'invert': True}),  # left middle
        ('1', {'joint_index': 4, 'side': 'left', 'min': -1.0, 'max': 0.0,
               'uppercase': '!', 'invert': True}),  # left ring
        ('`', {'joint_index': 'reset', 'side': 'left', 'uppercase': '~'}),

        ('6', {'joint_index': 0, 'side': 'right', 'min': 0.0, 'max': 1.8,
               'uppercase': '^'}),  # right thumb roll
        ('7', {'joint_index': 1, 'side': 'right', 'min': 0.0, 'max': 0.55,
               'uppercase': '&'}),  # right thumb pitch
        ('8', {'joint_index': 2, 'side': 'right', 'min': 0.0, 'max': 1.1,
               'uppercase': '*'}),  # right index
        ('9', {'joint_index': 3, 'side': 'right', 'min': 0.0, 'max': 0.9,
               'uppercase': '('}),  # right middle
        ('0', {'joint_index': 4, 'side': 'right', 'min': 0.0, 'max': 1.0,
               'uppercase': ')'}),  # right ring
        ('-', {'joint_index': 'reset', 'side': 'right', 'uppercase': '_'}),
    ])

    HEAD_BINDINGS = OrderedDict([
        ('x', {'joint_index': 0, 'min': -0.5, 'max': 0.5}),
        ('c', {'joint_index': 1, 'min': -0.5, 'max': 0.5}),
        ('v', {'joint_index': 2, 'min': -1.0, 'max': 1.0}),
        ('g', {'joint_index': 'reset'}),
    ])

    NECK_BINDINGS = OrderedDict([
        ('b', {'joint_index': 0, 'min': 0.0, 'max': 0.5}),  # lowerNeckPitch? 'min': 0, 'max': 1.162
        ('n', {'joint_index': 1, 'min': -1.0, 'max': 1.0}),  # neckYaw? 'min': -1.047, 'max': 1.047
        ('m', {'joint_index': 2, 'min': -0.5, 'max': 0.0}),  # upperNeckPitch? 'min': -0.872, 'max': 0
        ('h', {'joint_index': 'reset'}),
    ])

    WALKING_BINDINGS = OrderedDict([
        ('f', {'action': 'translate'}),
        ('z', {'action': 'rotate'}),
    ])

    def __init__(self):
        self.joint_values = {
            'left arm': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'right arm': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'left hand': [0.0, 0.0, 0.0, 0.0, 0.0],
            'right hand': [0.0, 0.0, 0.0, 0.0, 0.0],
            'head': [0.0, 0.0, 0.0],
            'neck': [0.0, 0.0, 0.0],
        }
        # ensure no characters are bound repeatedly
        bindings = [self.ARM_BINDINGS, self.HEAD_BINDINGS, self.NECK_BINDINGS]
        keys = set([])
        [keys.update(b.keys()) for b in bindings]
        assert len(keys) == sum([len(b) for b in bindings]), 'Duplicate binding'

    def run(self):
        try:
            self.init()
            self.print_usage()

            if (ON_REAL_ROBOT_USE):
                print 'Will send messages formatted for the real robot'                      
            else:
                print 'Will send messages formatted for sim'
            while not rospy.is_shutdown():
                ch = self.get_key()
                self.process_key(ch)
        finally:
            self.fini()

    def init(self):
        # save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)  # noqa
        self.footstep_count = 0
        # create publisher for arm trajectories
        robot_name = "valkyrie" #rospy.get_param('/ihmc_ros/robot_name')
        self.arm_publisher = rospy.Publisher(
            '/ihmc_ros/{0}/control/arm_trajectory'.format(robot_name),
            ArmTrajectoryRosMessage, queue_size=1)
        self.left_hand_publisher = rospy.Publisher(
            '/left_hand_position_controller/command',
            Float64MultiArray, queue_size=1)
        self.right_hand_publisher = rospy.Publisher(
            '/right_hand_position_controller/command',
            Float64MultiArray, queue_size=1)
        self.neck_publisher = rospy.Publisher(
            '/ihmc_ros/{0}/control/neck_trajectory'.format(robot_name),
            NeckTrajectoryRosMessage, queue_size=1)
        self.head_publisher = rospy.Publisher(
            '/ihmc_ros/{0}/control/head_trajectory'.format(robot_name),
            HeadTrajectoryRosMessage, queue_size=1)
        self.footstep_publisher = rospy.Publisher(
            '/ihmc_ros/{0}/control/footstep_list'.format(robot_name),
            FootstepDataListRosMessage, queue_size=1)
        self.footstep_status_subscriber = rospy.Subscriber(
            '/ihmc_ros/{0}/output/footstep_status'.format(robot_name),
            FootstepStatusRosMessage, self.receivedFootStepStatus_cb)
        self.abort_walking_publisher = rospy.Publisher(
            '/ihmc_ros/{0}/control/abort_walking'.format(robot_name),
            AbortWalkingRosMessage, queue_size=1)

        # Declare Footstep command Visualizer
        self.footstep_visualize_publisher = rospy.Publisher('/visualize_footstep_teleop', MarkerArray, queue_size=1)

        # right_foot_frame_parameter_name = "/ihmc_ros/{0}/right_foot_frame_name".format(robot_name)
        # left_foot_frame_parameter_name = "/ihmc_ros/{0}/left_foot_frame_name".format(robot_name)
        # if rospy.has_param(right_foot_frame_parameter_name) and \
        #         rospy.has_param(left_foot_frame_parameter_name):
        #     self.RIGHT_FOOT_FRAME_NAME = rospy.get_param(right_foot_frame_parameter_name)
        #     self.LEFT_FOOT_FRAME_NAME = rospy.get_param(left_foot_frame_parameter_name)
        # make sure the simulation is running otherwise wait
        self.rate = rospy.Rate(2)  # 2hz
        publishers = [self.neck_publisher]
#            self.arm_publisher, self.left_hand_publisher, self.right_hand_publisher,
#            self.neck_publisher, self.head_publisher, self.footstep_publisher]
        if any([p.get_num_connections() == 0 for p in publishers]):
            while any([p.get_num_connections() == 0 for p in publishers]):
                rospy.loginfo('waiting for subscribers: ' + ', '.join(sorted([
                    p.name for p in publishers if p.get_num_connections() == 0])))
                self.rate.sleep()

    def fini(self):
        self.arm_publisher = None

        # restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def print_usage(self):
        def get_bound_char(bindings, x):
            upper = bindings[x].get('uppercase', x.upper())
            return upper if bindings[x].get('invert', False) else x

        msg_args = []

        # left arm - first row
        msg_args.append(', '.join([
            get_bound_char(self.ARM_BINDINGS, x)
            for x in self.ARM_BINDINGS.keys()[0:4]]))
        # left arm - second row
        msg_args.append(', '.join([
            get_bound_char(self.ARM_BINDINGS, x)
            for x in self.ARM_BINDINGS.keys()[4:7]]))
        # left arm - reset
        msg_args.append(' or '.join([
            self.ARM_BINDINGS.keys()[7], self.ARM_BINDINGS.keys()[7].upper()]))

        # right arm - first row
        msg_args.append(', '.join([
            get_bound_char(self.ARM_BINDINGS, x)
            for x in self.ARM_BINDINGS.keys()[8:12]]))
        # right arm - second row
        msg_args.append(', '.join([
            get_bound_char(self.ARM_BINDINGS, x)
            for x in self.ARM_BINDINGS.keys()[12:15]]))
        # right arm - reset
        msg_args.append(' or '.join([
            self.ARM_BINDINGS.keys()[15], self.ARM_BINDINGS.keys()[15].upper()]))

        # left hand
        msg_args.append(', '.join([
            get_bound_char(self.HAND_BINDINGS, x)
            for x in self.HAND_BINDINGS.keys()[0:5]]))
        # left hand - reset
        msg_args.append(' or '.join([
            self.HAND_BINDINGS.keys()[5], self.HAND_BINDINGS.values()[5].get('uppercase')]))

        # right hand
        msg_args.append(', '.join([
            get_bound_char(self.HAND_BINDINGS, x)
            for x in self.HAND_BINDINGS.keys()[6:11]]))
        # right hand - reset
        msg_args.append(' or '.join([
            self.HAND_BINDINGS.keys()[11], self.HAND_BINDINGS.values()[11].get('uppercase')]))

        # head - roll, pitch, yaw
        msg_args += [
            get_bound_char(self.HEAD_BINDINGS, x)
            for x in self.HEAD_BINDINGS.keys()[0:3]]
        # head - reset
        msg_args.append(' or '.join([
            self.HEAD_BINDINGS.keys()[3], self.HEAD_BINDINGS.keys()[3].upper()]))

        # neck
        msg_args.append(', '.join([
            get_bound_char(self.NECK_BINDINGS, x)
            for x in self.NECK_BINDINGS.keys()[0:3]]))
        # neck - reset
        msg_args.append(' or '.join([
            self.NECK_BINDINGS.keys()[3], self.NECK_BINDINGS.keys()[3].upper()]))

        # walking
        [msg_args.append('{}'.format(x)) for x in self.WALKING_BINDINGS.keys()]

        print(msg_args)

        msg = """
            Keyboard Teleop for Space Robotics Challenge 0.1.0
            Copyright (C) 2017 Open Source Robotics Foundation
            Released under the Apache 2 License
            --------------------------------------------------
            Left arm joints: {}
                             {}
                Reset: {}

            Right arm joints: {}
                              {}
                Reset: {}

            Left hand joints: {}
                Reset: {}

            Right hand joints: {}
                Reset: {}

            Head angles:
                Roll: {}
                Pitch: {}
                Yaw: {}
                Reset all head angles: {}

            Neck joints: {}
                Reset all neck joints: {}

            The shown characters turn the joints in positive direction.
            The opposite case turns the joints in negative direction.
            For numbers the uppercase character is based on an English layout.

            Walking controls:
                Walk: {} lowercase meaning forward and uppercase backwards
                Rotating on the spot: {} lowercase is counterclockwise, uppercase is clockwise
            ?: Print this menu
            <TAB>: Print all current joint values
            <ESC>: Quit
            """.format(*msg_args)
        self.loginfo(msg)

    def get_key(self):
        """Get input from the terminal."""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        return key

    def process_key(self, ch):
        """Process key event."""
        if ch == '?':
            self.print_usage()
            return

        if ord(ch) == 9:  # TAB key
            for label in sorted(self.joint_values.keys()):
                self.loginfo(
                    'Current joint values for the %s: %s' %
                    (label, self.joint_values[label]))
            return

        if ord(ch) in (3, 27):  # Ctrl+C or ESCAPE key
            self.loginfo('Quitting')
            rospy.signal_shutdown('Shutdown')
            return

        if ch.lower() in self.ARM_BINDINGS:
            self.process_arm_command(self.ARM_BINDINGS[ch.lower()], ch)
            return

        if ch.lower() in self.HAND_BINDINGS:
            self.process_hand_command(self.HAND_BINDINGS[ch.lower()], ch)
            return
        for k, v in self.HAND_BINDINGS.items():
            if ch in v.get('uppercase'):
                self.process_hand_command(self.HAND_BINDINGS[k], ch)
                return

        if ch.lower() in self.HEAD_BINDINGS:
            self.process_head_command(self.HEAD_BINDINGS[ch.lower()], ch)
            return

        if ch.lower() in self.NECK_BINDINGS:
            self.process_neck_command(self.NECK_BINDINGS[ch.lower()], ch)
            return

        if ch.lower() in self.WALKING_BINDINGS:
            self.process_walking_command(self.WALKING_BINDINGS[ch.lower()], ch)
            return

    def process_arm_command(self, binding, ch):
        msg = ArmTrajectoryRosMessage()
        msg.unique_id = -1

        side = binding['side']
        if side == 'left':
            msg.robot_side = ArmTrajectoryRosMessage.LEFT
        elif side == 'right':
            msg.robot_side = ArmTrajectoryRosMessage.RIGHT
        else:
            assert False, "Unknown arm side '%s'" % side

        self._update_joint_values('%s arm' % side, binding, ch)
        self._append_trajectory_point_1d(
            msg, 1.0, self.joint_values['%s arm' % side])

        self.arm_publisher.publish(msg)

    def process_hand_command(self, binding, ch):
        msg = Float64MultiArray()

        side = binding['side']
        if side not in ('left', 'right'):
            assert False, "Unknown arm side '%s'" % side
        pub = getattr(self, '%s_hand_publisher' % side)

        self._update_joint_values('%s hand' % side, binding, ch)

        dim = MultiArrayDimension()
        dim.label = 'fingers'
        dim.size = 5
        dim.stride = 5
        msg.layout.dim = [dim]
        msg.layout.data_offset = 0
        msg.data = []
        for joint_value in self.joint_values['%s hand' % side]:
            msg.data.append(joint_value)

        pub.publish(msg)

    def process_head_command(self, binding, ch):
        msg = HeadTrajectoryRosMessage()
        msg.unique_id = -1
        self._update_joint_values('head', binding, ch)
        self._append_trajectory_point_so3(
            msg, 1.0, self.joint_values['head'])
        self.head_publisher.publish(msg)

    def process_neck_command(self, binding, ch):
        msg = NeckTrajectoryRosMessage()
        msg.unique_id = -1
        self._update_joint_values('neck', binding, ch)
        self._append_trajectory_point_1d(
            msg, 1.0, self.joint_values['neck'])
        self.neck_publisher.publish(msg)

    def process_walking_command(self, binding, ch):
        is_lower_case = ch == ch.lower()
        # ROT_STEP = 90*3.14159/180.
        direction = 1.0 if is_lower_case else -1.0
        self.footstep_count = 0
        if binding['action'] == 'translate':
            self.loginfo('Walking ' + ('forward' if is_lower_case else 'backward'))
            self.translate([self.TRANS_STEP * direction, 0.0, 0.0])
        elif binding['action'] == 'rotate':
            self.loginfo('Rotating ' + ('counter-clockwise' if is_lower_case else 'clockwise'))
            self.rotate(self.ROT_STEP * direction)


    def get_pelvis_xy_coplanar_quat(self, q_pelvis_in):
        q_pelvis = quat.Normalize(q_pelvis_in)
        # Convert Quat to R matrix
        R_pelvis = quat.quat_to_R(q_pelvis)

        # Get Columns of R matrix
        pelvis_x_dir = R_pelvis[0][:]
        pelvis_y_dir = R_pelvis[1][:]
        pelvis_z_dir = R_pelvis[2][:]       

        # Zero out the z's of x and y
        pelvis_x_dir[2] = 0.0
        pelvis_y_dir[2] = 0.0        

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

    def createRotationFootStepList(self, yaw):
        left_footstep = FootstepDataRosMessage()
        left_footstep.robot_side = FootstepDataRosMessage.LEFT
        right_footstep = FootstepDataRosMessage()
        right_footstep.robot_side = FootstepDataRosMessage.RIGHT

        left_foot_world = self.tfBuffer.lookup_transform(
            'world', self.LEFT_FOOT_FRAME_NAME, rospy.Time())
        right_foot_world = self.tfBuffer.lookup_transform(
            'world', self.RIGHT_FOOT_FRAME_NAME, rospy.Time())  

        intermediate_transform = Transform()
        # create a virtual fram between the feet, this will be the center of the rotation
        intermediate_transform.translation.x = (
            left_foot_world.transform.translation.x + right_foot_world.transform.translation.x)/2.
        intermediate_transform.translation.y = (
            left_foot_world.transform.translation.y + right_foot_world.transform.translation.y)/2.
        intermediate_transform.translation.z = (
            left_foot_world.transform.translation.z + right_foot_world.transform.translation.z)/2.
        # here we assume that feet have the same orientation so we can pick arbitrary left or right
        #intermediate_transform.rotation = left_foot_world.transform.rotation #  Get Pelvis orientation


        # We will use the pelvis's rotation as the initial frame of reference
        pelvis_world = self.tfBuffer.lookup_transform(
            'world', self.PELVIS_FRAME_NAME, rospy.Time())                

        quat_pelvis_world = pelvis_world.transform.rotation
        quat_pelvis = numpy.array([quat_pelvis_world.w, quat_pelvis_world.x, quat_pelvis_world.y, quat_pelvis_world.z])
        quat_to_use = self.get_pelvis_xy_coplanar_quat(quat_pelvis)

        quat = Quaternion()
        quat.w = quat_to_use[0]
        quat.x = quat_to_use[1]
        quat.y = quat_to_use[2]
        quat.z = quat_to_use[3]
        intermediate_transform.rotation = quat      

        left_footstep.location = left_foot_world.transform.translation
        right_footstep.location = right_foot_world.transform.translation

        # define the turning radius
        radius = sqrt(
            (
                right_foot_world.transform.translation.x -
                left_foot_world.transform.translation.x
            )**2 + (
                right_foot_world.transform.translation.y -
                left_foot_world.transform.translation.y
            )**2) / 2.

        left_offset = [-radius*sin(yaw), radius*(1-cos(yaw)), 0]
        right_offset = [radius*sin(yaw), -radius*(1-cos(yaw)), 0]
        intermediate_euler = euler_from_quaternion([
            intermediate_transform.rotation.x,
            intermediate_transform.rotation.y,
            intermediate_transform.rotation.z,
            intermediate_transform.rotation.w])
        resulting_quat = quaternion_from_euler(
            intermediate_euler[0], intermediate_euler[1],
            intermediate_euler[2] + yaw)

        rot = quaternion_matrix([
            resulting_quat[0], resulting_quat[1], resulting_quat[2], resulting_quat[3]])
        left_transformedOffset = numpy.dot(rot[0:3, 0:3], left_offset)
        right_transformedOffset = numpy.dot(rot[0:3, 0:3], right_offset)
        quat_final = Quaternion(
            resulting_quat[0], resulting_quat[1], resulting_quat[2], resulting_quat[3])

        left_footstep.location.x += left_transformedOffset[0]
        left_footstep.location.y += left_transformedOffset[1]

        if ON_REAL_ROBOT_USE:
            left_footstep.location.z = 0.0 
        else:
            left_footstep.location.z = -SOLE_FRAME_Z_OFFSET #+= left_transformedOffset[2]            
        left_footstep.orientation = quat_final

        right_footstep.location.x += right_transformedOffset[0]
        right_footstep.location.y += right_transformedOffset[1]
        if ON_REAL_ROBOT_USE:
            right_footstep.location.z = 0.0 
        else:
            right_footstep.location.z = -SOLE_FRAME_Z_OFFSET #+= right_transformedOffset[2]
        right_footstep.orientation = quat_final

        if yaw > 0:
            return [left_footstep, right_footstep]
        else:
            return [right_footstep, left_footstep]

    # Creates footstep with the current position and orientation of the foot.
    def createFootStepInPlace(self, step_side):
        footstep = FootstepDataRosMessage()
        footstep.robot_side = step_side

        if step_side == FootstepDataRosMessage.LEFT:
            foot_frame = self.LEFT_FOOT_FRAME_NAME
        else:
            foot_frame = self.RIGHT_FOOT_FRAME_NAME

        footWorld = self.tfBuffer.lookup_transform('world', foot_frame, rospy.Time())


        #Get Pelvis orientation as we want to be with respect to the pelvis
        pelvis_world = self.tfBuffer.lookup_transform(
            'world', self.PELVIS_FRAME_NAME, rospy.Time())                

        quat_pelvis_world = pelvis_world.transform.rotation
        quat_pelvis = numpy.array([quat_pelvis_world.w, quat_pelvis_world.x, quat_pelvis_world.y, quat_pelvis_world.z])
        quat_to_use = self.get_pelvis_xy_coplanar_quat(quat_pelvis)

        quat = Quaternion()
        quat.w = quat_to_use[0]
        quat.x = quat_to_use[1]
        quat.y = quat_to_use[2]
        quat.z = quat_to_use[3]

        footstep.orientation = quat #footWorld.transform.rotation
        footstep.location = footWorld.transform.translation

        return footstep

    # Creates footstep offset from the current foot position. The offset is in foot frame.
    def createTranslationFootStepOffset(self, step_side, offset):
        footstep = self.createFootStepInPlace(step_side)

        # transform the offset to world frame
        #quat = footstep.orientation 
        
        #Get Pelvis orientation as we want to be with respect to the pelvis
        pelvis_world = self.tfBuffer.lookup_transform(
            'world', self.PELVIS_FRAME_NAME, rospy.Time())                

        quat_pelvis_world = pelvis_world.transform.rotation
        quat_pelvis = numpy.array([quat_pelvis_world.w, quat_pelvis_world.x, quat_pelvis_world.y, quat_pelvis_world.z])
        quat_to_use = self.get_pelvis_xy_coplanar_quat(quat_pelvis)

        quat = Quaternion()
        quat.w = quat_to_use[0]
        quat.x = quat_to_use[1]
        quat.y = quat_to_use[2]
        quat.z = quat_to_use[3]

        rot = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
        transformedOffset = numpy.dot(rot[0:3, 0:3], offset)

        footstep.location.x += transformedOffset[0]
        footstep.location.y += transformedOffset[1]
        if (ON_REAL_ROBOT_USE):
            footstep.location.z = 0.0 #+= transformedOffset[2]
        else:
            footstep.location.z = -SOLE_FRAME_Z_OFFSET #+= transformedOffset[2]            
        #print "(left, right)", self.LEFT_FOOT_FRAME_NAME, self.RIGHT_FOOT_FRAME_NAME
        #print "(x,y,z)", footstep.location.x, footstep.location.y, footstep.location.z

        return footstep

    def getEmptyFootsetListMsg(self):
        msg = FootstepDataListRosMessage()
        if ON_REAL_ROBOT_USE:
            msg.execution_timing = msg.CONTROL_DURATIONS
            msg.default_transfer_duration = 0.0 # IHMC default param
            msg.default_swing_duration = 0.0 # IHMC default param
            msg.final_transfer_duration = -9.81
        else:
            msg.default_transfer_time = 0.0 #3.0#1.5
            msg.default_swing_time = 0.0 #3.0#1.5
        msg.execution_mode = 0
        msg.unique_id = -1
        return msg

    def getTranslationFootstepMsg(self, offset):
        msg = self.getEmptyFootsetListMsg()
        msg.footstep_data_list.append(self.createTranslationFootStepOffset(
            FootstepDataRosMessage.LEFT, offset))
        msg.footstep_data_list.append(self.createTranslationFootStepOffset(
            FootstepDataRosMessage.RIGHT, offset))
        return msg

    def getRotationFooststepMsg(self, yaw):
        msg = self.getEmptyFootsetListMsg()
        footsteps = self.createRotationFootStepList(yaw)
        for footstep in footsteps:
            msg.footstep_data_list.append(footstep)

        return msg

    def createMarker(self, marker_id, position, orientation):
        marker_msg = Marker()
        marker_msg.header.frame_id = "world";
        marker_msg.header.stamp = rospy.Time();

        if (marker_id == 0):
            marker_msg.action = marker_msg.DELETE
            return marker_msg
        marker_msg.ns = "visualize_footstep_cmds";
        marker_msg.id = marker_id

        marker_msg.type = marker_msg.MESH_RESOURCE;
        marker_msg.mesh_resource = MESH_LOCATION #"package://val_description/model/meshes/legs/foot.dae"
        marker_msg.action = marker_msg.ADD
        marker_msg.pose.position.x = position.x
        marker_msg.pose.position.y = position.y
        if ON_REAL_ROBOT_USE:        
            marker_msg.pose.position.z = position.z - SOLE_FRAME_Z_OFFSET            
        else:
            marker_msg.pose.position.z = position.z        
        marker_msg.pose.orientation = orientation

        self.loginfo("Marker (x,y,z) (" + str(position.x) + ", " + str(position.y) + ", " + str(position.z) + ")")

        marker_msg.scale.x = 1.0;
        marker_msg.scale.y = 1.0;
        marker_msg.scale.z = 1.0;        

        # marker_msg.color.r = 0.0
        # marker_msg.color.g = 1.0
        # marker_msg.color.b = 0.0
        # marker_msg.color.a = 0.5
        marker_msg.mesh_use_embedded_materials = True

        marker_msg.lifetime = rospy.Duration()
        return marker_msg


    # // Set the namespace and id for this marker.  This serves to create a unique ID
    # // Any marker sent with the same namespace and id will overwrite the old one
    # marker.ns = "basic_shapes";
    # marker.id = 0;

    # // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    # marker.type = shape;

    # // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    # marker.action = visualization_msgs::Marker::ADD;

    # // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    # marker.pose.position.x = 0;
    # marker.pose.position.y = 0;
    # marker.pose.position.z = 0;
    # marker.pose.orientation.x = 0.0;
    # marker.pose.orientation.y = 0.0;
    # marker.pose.orientation.z = 0.0;
    # marker.pose.orientation.w = 1.0;

    # // Set the scale of the marker -- 1x1x1 here means 1m on a side
    # marker.scale.x = 1.0;
    # marker.scale.y = 1.0;
    # marker.scale.z = 1.0;

    # // Set the color -- be sure to set alpha to something non-zero!
    # marker.color.r = 0.0f;
    # marker.color.g = 1.0f;
    # marker.color.b = 0.0f;
    # marker.color.a = 1.0;

    # marker.lifetime = ros::Duration();


    def visualize_footsteps(self, msg):
        markerArray_msg = MarkerArray()
        number_of_footsteps = len(msg.footstep_data_list)
        self.loginfo("Number of Footsteps " + str(number_of_footsteps))
        self.loginfo("Publishing Marker Array")

        markerArray_msg.markers.append(self.createMarker(0, None, None) ) #Delete old markers
        marker_id = 1
        for footstep_msg in msg.footstep_data_list:
            markerArray_msg.markers.append(self.createMarker(marker_id, footstep_msg.location, footstep_msg.orientation) )
            marker_id += 1

        self.footstep_visualize_publisher.publish(markerArray_msg)

    def execute_footsteps(self, msg):
        self.footstep_count = 0
        self.visualize_footsteps(msg)        

        if (EXECUTE_FOOTSTEPS):
            self.footstep_publisher.publish(msg)
            number_of_footsteps = len(msg.footstep_data_list)
            max_iterations = 100
            count = 0
            while count < max_iterations:
                self.rate.sleep()
                count += 1
                if self.footstep_count == number_of_footsteps:
                    return True
                    break
            msg = AbortWalkingRosMessage()
            msg.unique_id = -1
            self.abort_walking_publisher.publish(msg)
            return False
        else:
            return True

    def translate(self, offset):
        self.set_init_pose()
        msg = self.getTranslationFootstepMsg(offset)
        res = self.execute_footsteps(msg)
        if res:
            self.loginfo('done walking')
            return
        self.loginfo('failed to walk, aborting trajectory')

    def rotate(self, yaw):
        # space feet further apart if not spaced enough for safe rotation
        self.set_init_pose()
        msg = self.getRotationFooststepMsg(yaw)
        res = self.execute_footsteps(msg)
        if res:
            self.loginfo('done rotating')
            return
        self.loginfo('failed to rotate, aborting trajectory')

    def set_init_pose(self):
        left_foot_world = self.tfBuffer.lookup_transform(
            'world', self.LEFT_FOOT_FRAME_NAME, rospy.Time())
        right_foot_world = self.tfBuffer.lookup_transform(
            'world', self.RIGHT_FOOT_FRAME_NAME, rospy.Time())
        intermediate_transform = Transform()
        distance_between_feet = sqrt(
            (
                right_foot_world.transform.translation.x -
                left_foot_world.transform.translation.x
            )**2 + (
                right_foot_world.transform.translation.y -
                left_foot_world.transform.translation.y
            )**2)
        if distance_between_feet <= 0.2:
            self.loginfo('moving feet further apart\n')
            msg = self.getEmptyFootsetListMsg()
            msg.footstep_data_list.append(self.createTranslationFootStepOffset(
                FootstepDataRosMessage.LEFT, [0.0, 0.075, 0.0]))
            msg.footstep_data_list.append(self.createTranslationFootStepOffset(
                FootstepDataRosMessage.RIGHT, [0.0, -0.075, 0.0]))
            self.execute_footsteps(msg)
            self.loginfo('done moving feet further apart\n')

    def _update_joint_values(self, label, binding, ch):
        joint_index = binding['joint_index']
        if joint_index == 'reset':
            # reset all joints to zero
            self.loginfo('Reset %s' % label)
            self.joint_values[label] = [0.0] * len(self.joint_values[label])

        else:
            # update value within boundaries
            value = self.joint_values[label][joint_index]
            upper = binding.get('uppercase', ch.upper())
            is_lower_case = ch != upper
            factor = 1.0 if is_lower_case else -1.0
            if binding.get('invert', False):
                factor *= -1.0
            STEP = 0.1
            value += STEP * factor
            value = min(max(value, binding['min']), binding['max'])
            self.joint_values[label][joint_index] = value

            self.loginfo(
                'Move %s joint #%d %s to %.1f: [%s]' %
                (label, joint_index, '++' if (is_lower_case != binding.get('invert', False)) else '--',
                 self.joint_values[label][joint_index],
                 ', '.join(
                     ['%.1f' % v for v in self.joint_values[label]])))

    def _append_trajectory_point_1d(self, msg, time, joint_values):
        if not msg.joint_trajectory_messages:
            msg.joint_trajectory_messages = [
                OneDoFJointTrajectoryRosMessage() for _ in joint_values]

        if ON_REAL_ROBOT_USE:
            for odjTm in msg.joint_trajectory_messages:
                odjTm.weight = float('NaN')

        for i, joint_value in enumerate(joint_values):
            point = TrajectoryPoint1DRosMessage()
            point.time = time
            point.position = joint_value
            point.velocity = 0
            msg.joint_trajectory_messages[i].trajectory_points.append(point)

    def _append_trajectory_point_so3(self, msg, time, joint_values):
        roll, pitch, yaw = joint_values
        quat = quaternion_from_euler(roll, pitch, yaw)
        point = SO3TrajectoryPointRosMessage()
        point.time = time
        point.orientation = Quaternion()
        point.orientation.x = quat[0]
        point.orientation.y = quat[1]
        point.orientation.z = quat[2]
        point.orientation.w = quat[3]
        point.angular_velocity = Vector3()
        point.angular_velocity.x = 0
        point.angular_velocity.y = 0
        point.angular_velocity.z = 0
        msg.taskspace_trajectory_points.append(point)

    def loginfo(self, msg):
        """Log info message while terminal is in funky mode."""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        rospy.loginfo(msg)
        tty.setraw(sys.stdin.fileno())


if __name__ == '__main__':
    rospy.init_node('keyboard_teleop')

    # parser = argparse.ArgumentParser()
    # parser.add_argument('-r', '--real-robot',  help='Pass "True" if running on the real robot. argument is set to "False" otherwise', default="False")
    # args = parser.parse_args()
    # print args.real_robot

    # if (args.real_robot == "True"):
    #     ON_REAL_ROBOT_USE = True
    # else:
    #     ON_REAL_ROBOT_USE = False

    teleop = KeyboardTeleop()
    teleop.run()
