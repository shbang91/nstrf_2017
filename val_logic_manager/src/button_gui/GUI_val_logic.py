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


# ----- Start ------
import rospy
import sys
#import rospkg
import yaml
from std_msgs.msg import String
from std_msgs.msg import Int8
from PyQt4 import QtGui, QtCore

class ValGui(QtGui.QWidget):

  def __init__(self):
      QtGui.QWidget.__init__(self)
      newFont = QtGui.QFont("Helvetica", 18, QtGui.QFont.Bold)

      # Add a main layout
      mainLayout = QtGui.QVBoxLayout(self)
      #mainLayout->setMeubBar(menuBar)
      # Add buttons with the commands
      grid = QtGui.QGridLayout()
      grid.setSpacing(20)

      # Initialize rosnode
      rospy.init_node("simple_val_gui") 

      #rospack = rospkg.RosPack()
      default_pub_topic = GUI_CMD_TOPIC

      # Set Commands
      self.commands = [GO_HOME_GUI_STRING,
                       SEND_SINGLE_IK_GUI_STRING, 
                       RE_INIT_MARKERS_GUI_STRING,
                       RUN_GRASPLOC_GUI_STRING
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
      self.move(400,400)
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
      else:
        string_cmd = INVALID_CMD
      
      rospy.loginfo(command)

      if send_command:
        msg = String()
        msg.data = string_cmd
        self.pub.publish(msg)

def gui_start():
    app = QtGui.QApplication(sys.argv)
    sg = ValGui()
    sys.exit(app.exec_())


gui_start()
