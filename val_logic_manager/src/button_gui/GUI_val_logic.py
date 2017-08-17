#!/usr/bin/env python

# Val simple GUI
# Based on Andrea Thomaz's lab's class code in UT Austin
# Modified for personal use. 
#     stevenjj@utexas.edu

#from GUI_params import *

# GUI Command list
GUI_CMD_TOPIC = 'val_logic_manager/operator_command'
INVALID_CMD = "invalid_cmd"

GO_HOME, GO_HOME_GUI_STRING  = "go_home", "Go Neutral Pos"
SEND_SINGLE_IK,  SEND_SINGLE_IK_GUI_STRING   = "send_single_ik", "Send Single IK"
RE_INIT_MARKERS, RE_INIT_MARKERS_GUI_STRING = "re_init_markers", "Re Initialize Markers"

RUN_GRASPLOC, RUN_GRASPLOC_GUI_STRING = "run_grasploc", "Run Grasploc"
GET_NEAREST_GRASP_IK, GET_NEAREST_GRASP_IK_GUI_STRING = "get_nearest_grasp_ik", "Get Nearest Grasp IK"
TRY_NEXT_GRASP_IK, TRY_NEXT_GRASP_IK_GUI_STRING = "try_next_grasp_ik", "Try IK for Next Grasp"

USE_RIGHT_HAND, USE_RIGHT_HAND_GUI_STRING = "use_right_hand", "Use Right Hand"
USE_LEFT_HAND, USE_LEFT_HAND_GUI_STRING = "use_left_hand", "Use Left Hand"
STOP_ALL_TRAJECTORIES, STOP_ALL_TRAJECTORIES_GUI_STRING = "stop_all_trajectories", "Stop All Trajectories"


# ----- Start ------
import signal
import sys


import rospy
#import rospkg
import yaml
from std_msgs.msg import String
from std_msgs.msg import Int8
from PyQt4.QtCore import QTimer
from PyQt4 import QtGui, QtCore

class ValGui(QtGui.QWidget):

  def __init__(self):
      QtGui.QWidget.__init__(self)
      self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)
      newFont = QtGui.QFont("Helvetica", 14, QtGui.QFont.Bold)

      # Add a main layout
      mainLayout = QtGui.QVBoxLayout(self)
      #mainLayout->setMeubBar(menuBar)
      # Add buttons with the commands
      grid = QtGui.QGridLayout()
      grid.setSpacing(3)

      # Initialize rosnode
      rospy.init_node("steven_simple_val_gui") 

      #rospack = rospkg.RosPack()
      default_pub_topic = GUI_CMD_TOPIC

      # Set Commands
      self.commands = [GO_HOME_GUI_STRING,
                       SEND_SINGLE_IK_GUI_STRING, 
                       RE_INIT_MARKERS_GUI_STRING,
                       RUN_GRASPLOC_GUI_STRING,
                       GET_NEAREST_GRASP_IK_GUI_STRING,
                       TRY_NEXT_GRASP_IK_GUI_STRING,
                       USE_RIGHT_HAND_GUI_STRING,
                       USE_LEFT_HAND_GUI_STRING,
                       STOP_ALL_TRAJECTORIES_GUI_STRING
                       ] 
      
      positions = [(i,j) for i in range(len(self.commands)) for j in range(3)]
           
      for position, name in zip(positions, self.commands):
          button = QtGui.QPushButton(name)
          button.setObjectName('%s' % name)
          button.setFont(newFont)
          button.setStyleSheet("background-color: #FFA500")
          button.clicked.connect(self.handleButton)
          grid.addWidget(button, *position)

      mainLayout.addLayout(grid)
      mainLayout.addStretch()
      
      # Show the GUI 
      self.adjustSize()
      self.setWindowTitle("GUI Val Logic")
      self.move(400,100)
      self.show()
      self.raise_()

      # # Create the publisher to publish the commands to
      self.pub = rospy.Publisher(default_pub_topic, String, queue_size=1)

      rospy.loginfo("Finished initializing GUI Val Logic")

  # Button handler after its clicked
  def handleButton(self):
      clicked_button = self.sender()

      string_cmd = INVALID_CMD
      send_command = False
      # # Publish everytime a command is selected from the combo box
      command = str(clicked_button.objectName())

      if command in self.commands:
        send_command = True

      if command == GO_HOME_GUI_STRING: 
        string_cmd = GO_HOME
      elif command == SEND_SINGLE_IK_GUI_STRING: 
        string_cmd = SEND_SINGLE_IK 
      elif command == RE_INIT_MARKERS_GUI_STRING: 
        string_cmd = RE_INIT_MARKERS      
      elif command == RUN_GRASPLOC_GUI_STRING: 
        string_cmd = RUN_GRASPLOC                    
      elif command == GET_NEAREST_GRASP_IK_GUI_STRING: 
        string_cmd = GET_NEAREST_GRASP_IK                    
      elif command == TRY_NEXT_GRASP_IK_GUI_STRING: 
        string_cmd = TRY_NEXT_GRASP_IK                             
      elif command == USE_RIGHT_HAND_GUI_STRING: 
        string_cmd = USE_RIGHT_HAND
      elif command == USE_LEFT_HAND_GUI_STRING: 
        string_cmd = USE_LEFT_HAND
      elif command == STOP_ALL_TRAJECTORIES_GUI_STRING:
        string_cmd = STOP_ALL_TRAJECTORIES        
      else:
        string_cmd = INVALID_CMD
      
      rospy.loginfo(command)

      if send_command:
        msg = String()
        msg.data = string_cmd
        self.pub.publish(msg)

# def gui_start():
#     app = QtGui.QApplication(sys.argv)
#     sg = ValGui()
#     sys.exit(app.exec_())


def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    QtGui.QApplication.quit()

if __name__ == "__main__":
  #gui_start
  signal.signal(signal.SIGINT, sigint_handler)

  app = QtGui.QApplication(sys.argv)
  timer = QTimer()
  timer.start(100)  # You may change this if you wish.
  timer.timeout.connect(lambda: None)  # Let the interpreter run each 100 ms.

  sg = ValGui()
  sys.exit(app.exec_())

