import sys
import rospy

import tf2_ros
from ihmc_msgs.msg import HandTrajectoryRosMessage
from ihmc_msgs.msg import SE3TrajectoryPointRosMessage
from geometry_msgs.msg import Quaternion, Transform, Vector3, Point

WORLD_FRAME_HASH = 83766130 # REAL_ROBOT_ONLY

# HandTrajectoryRosMessage msg
# # 
# robot_side = msg.RIGHT

# # REAL ROBOT ONLY
# frame_information.trajectory_reference_frame_id = WORLD_FRAME_HASH;
# frame_information.data_reference_frame_id = WORLD_FRAME_HASH;         
# use_custom_control_frame = false;
# # END REAL_ROBOT_ONLY
#
# SIM_ONLY
# base_for_control = msg.WORLD



ON_REAL_ROBOT = False

DESIRED_HAND = 1 # Left = 0, Right = 1 
DESIRED_HAND_POS = Point()  # Right hand Sim Start Pos: (0.304, -0.242, 0.937)
DESIRED_HAND_ORI = Quaternion() # Right hand Sim Start Orientation: (-0.03134541605123229, 0.09557238644513989, 0.8871963617792723, 0.4502954579912186)

# Val will raise her hand and bring it to the front of her chest
dx = 0.10
dy = 0.04
dz = 0.25

# estimated and actual hand positions are slightly different.
DESIRED_HAND_POS.x = 0.304 + dx
DESIRED_HAND_POS.y = -0.242 + dy
DESIRED_HAND_POS.z = 0.937 + dz

DESIRED_HAND_ORI.x = -0.0313
DESIRED_HAND_ORI.y = 0.0955
DESIRED_HAND_ORI.z = 0.8871
DESIRED_HAND_ORI.w = 0.4502

class SE3_object():
    def init(self):
        self.robot_name = "valkyrie"
        self.right_palm_frame_name = "rightPalm"
        self.left_palm_frame_name = "leftPalm"        

        self.tfBuffer = tf2_ros.Buffer()        
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)  #
        self.hand_traj_pub = rospy.Publisher('/ihmc_ros/{0}/control/hand_trajectory'.format(self.robot_name), HandTrajectoryRosMessage, queue_size=1)

        self.rate = rospy.Rate(10)  # 10 Hz


    def run(self):
        self.init()
        while not rospy.is_shutdown():
            rospy.loginfo('Node running at 10 Hz')
            self.rate.sleep()
            
            # Test hand pose lookup
            # self.get_current_hand_se3(0) # LEFT
            # self.get_current_hand_se3(1) # RIGHT            

            # Test sender here            
            self.publish_once_and_kill()




    def get_current_hand_se3(self, robot_side):
        hand_name = None
        hand = None
        if robot_side == 0: # LEFT
            hand_name = "LEFT"
            while True:
                try:
                    hand = self.tfBuffer.lookup_transform('world', self.left_palm_frame_name, rospy.Time())
                    break
                except:
                    print "Could not get Left Palm Position"
                    print "    Trying again..."
                    self.rate.sleep()


        elif robot_side == 1: # RIGHT
            hand_name = "Right"
            while True:
                try:
                    hand = self.tfBuffer.lookup_transform('world', self.right_palm_frame_name, rospy.Time())
                    break
                except:
                    print "Could not get Right Palm Position"
                    print "Could not get Left Palm Position"
                    print "    Trying again..."
                    self.rate.sleep()

        else:
            raise 'Error. robot_side not specified properly'
        
        hand_transform = hand #Transform()
        hand_position = Vector3()
        if ON_REAL_ROBOT:
            hand_position = Point()

        hand_orientation = Quaternion()

        hand_position.x = hand_transform.transform.translation.x
        hand_position.y = hand_transform.transform.translation.y        
        hand_position.z = hand_transform.transform.translation.z

        hand_orientation.x = hand_transform.transform.rotation.x
        hand_orientation.y = hand_transform.transform.rotation.y
        hand_orientation.z = hand_transform.transform.rotation.z
        hand_orientation.w = hand_transform.transform.rotation.w

        print "  Actual Hand Pose"
        print "     " , hand_name, "Hand Pos (x,y,z)",    (hand_position.x, hand_position.y, hand_position.z)
        print "     " , hand_name, "Ori (x,y,z,w)",  (hand_orientation.x, hand_orientation.y, hand_orientation.z, hand_orientation.w)

        return hand_position, hand_orientation



    def publish_once_and_kill(self):
        msg = HandTrajectoryRosMessage()
        msg.robot_side = DESIRED_HAND #msg.RIGHT
        msg.execution_mode = msg.OVERRIDE
        msg.unique_id = 2

        if ON_REAL_ROBOT:
            # REAL ROBOT ONLY
            print 'On real Robot'
            msg.frame_information.trajectory_reference_frame_id = WORLD_FRAME_HASH;
            msg.frame_information.data_reference_frame_id = WORLD_FRAME_HASH;         
            msg.use_custom_control_frame = 0;
            # END REAL_ROBOT_ONLY
        else:
            base_for_control = msg.WORLD

        se3_final = SE3TrajectoryPointRosMessage()            

        # Set Zero Velocity for Linear and Angular components
        zero_vel = Vector3()
        zero_vel.x = 0.0
        zero_vel.y = 0.0
        zero_vel.z = 0.0

        se3_final.time = 3.0 # Trajectory Time
        se3_final.position.x = DESIRED_HAND_POS.x
        se3_final.position.y = DESIRED_HAND_POS.y
        se3_final.position.z = DESIRED_HAND_POS.z 

        se3_final.orientation.x = DESIRED_HAND_ORI.x
        se3_final.orientation.y = DESIRED_HAND_ORI.y
        se3_final.orientation.z = DESIRED_HAND_ORI.z
        se3_final.orientation.w = DESIRED_HAND_ORI.w                
        se3_final.unique_id = msg.unique_id        

        se3_final.linear_velocity = zero_vel
        se3_final.angular_velocity = zero_vel        


        #msg.taskspace_trajectory_points.append(se3_init)
        msg.taskspace_trajectory_points.append(se3_final)        

        print 'Sending Hand Command in World Frame...'
        hand_name = "Right"
        if DESIRED_HAND == 0: # LEFT
            hand_name = "Left"
        print "     " , hand_name, "Des Hand Pos (x,y,z)",    (DESIRED_HAND_POS.x, DESIRED_HAND_POS.y, DESIRED_HAND_POS.z)
        print "     " , hand_name, "Des Ori (x,y,z,w)",  (DESIRED_HAND_ORI.x, DESIRED_HAND_ORI.y, DESIRED_HAND_ORI.z, DESIRED_HAND_ORI.w)

        # Get Current Hand Position and Orientation
        current_hand_pos, current_hand_ori = self.get_current_hand_se3(DESIRED_HAND)

        self.hand_traj_pub.publish(msg)
        self.rate.sleep()
        sys.exit()


if __name__ == '__main__':
    rospy.init_node('example_se3_message')
    sample_SE3_sender = SE3_object()
    sample_SE3_sender.run()
    