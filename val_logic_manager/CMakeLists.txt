cmake_minimum_required(VERSION 2.8.3)
project(val_logic_manager)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  message_filters
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  val_ik_msgs
  openpose_ros_msgs
  srcsim
  val_ik
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS   roscpp
	  rospy
	  roslib
	  message_filters
	  sensor_msgs
	  nav_msgs
	  geometry_msgs
	  visualization_msgs
	  val_ik_msgs
	  openpose_ros_msgs
	  srcsim
	  val_ik
	  tf
)

###########
## Build ##
###########
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)


add_executable(logic_main src/logic_main.cpp src/logic_callbacks.cpp src/logic_manager.cpp)
target_link_libraries(logic_main ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(logic_main ${catkin_EXPORTED_TARGETS})
