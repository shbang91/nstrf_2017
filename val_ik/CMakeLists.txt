cmake_minimum_required(VERSION 2.8.3)
project(val_ik)

set(DRAKE_CATKIN_WS /home/stevenjj/dev/drake_catkin_workspace)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  visualization_msgs  
  message_generation
  message_filters
  roslib
  std_msgs
  tf
  cmake_modules
)


#add_service_files(
#   FILES
#)

#generate_messages(
#  DEPENDENCIES
#  sensor_msgs
#  std_msgs
#)

find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  	roscpp 
  	message_runtime
    rospy
  	sensor_msgs
  	std_msgs
  DEPENDS
  	Eigen
  	roscpp
	system_lib
	tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${DRAKE_CATKIN_WS}/install/include
  ${DRAKE_CATKIN_WS}/install/include/lcmtypes

)

FIND_LIBRARY(DRAKECOMMON_LIB drakeCommon ${DRAKE_CATKIN_WS}/install/lib)
FIND_LIBRARY(DRAKEIK_LIB drakeIK ${DRAKE_CATKIN_WS}/install/lib)
FIND_LIBRARY(DRAKELCM_LIB drakeLcm ${DRAKE_CATKIN_WS}/install/lib)
FIND_LIBRARY(DRAKEMULTIBODYPARSERS_LIB drakeMultibodyParsers ${DRAKE_CATKIN_WS}/install/lib)
FIND_LIBRARY(DRAKERBM_LIB drakeRBM ${DRAKE_CATKIN_WS}/install/lib)
FIND_LIBRARY(DRAKERIGIDBODYPLANT_LIB drakeRigidBodyPlant ${DRAKE_CATKIN_WS}/install/lib)

FIND_LIBRARY(drakeLcmSystem_LIB drakeLcmSystem ${DRAKE_CATKIN_WS}/install/lib)
FIND_LIBRARY(drake_lcmtypes_LIB drake_lcmtypes ${DRAKE_CATKIN_WS}/install/lib)

FIND_LIBRARY(drakeJoints_LIB drakeJoints ${DRAKE_CATKIN_WS}/install/lib)
FIND_LIBRARY(drakeIKoptions_LIB drakeIKoptions ${DRAKE_CATKIN_WS}/install/lib)
FIND_LIBRARY(drakeRigidBodyConstraint_LIB drakeRigidBodyConstraint ${DRAKE_CATKIN_WS}/install/lib)


add_executable(val_ik src/val_ik.cpp)
target_link_libraries(val_ik ${catkin_LIBRARIES} ${DRAKECOMMON_LIB} 
												 ${DRAKEIK_LIB}
												 ${DRAKELCM_LIB} 
												 ${DRAKEMULTIBODYPARSERS_LIB} 
												 ${DRAKERBM_LIB}
												 ${DRAKERIGIDBODYPLANT_LIB}
												 ${drakeLcmSystem_LIB}
												 ${drake_lcmtypes_LIB}
												 ${drakeJoints_LIB}
												 ${drakeIKoptions_LIB}
												 ${drakeRigidBodyConstraint_LIB})

add_executable(val_ik_ros_pub_test src/val_ik_ros_pub_test.cpp)
target_link_libraries(val_ik_ros_pub_test ${catkin_LIBRARIES} ${DRAKECOMMON_LIB} 
                         ${DRAKEIK_LIB}
                         ${DRAKELCM_LIB} 
                         ${DRAKEMULTIBODYPARSERS_LIB} 
                         ${DRAKERBM_LIB}
                         ${DRAKERIGIDBODYPLANT_LIB}
                         ${drakeLcmSystem_LIB}
                         ${drake_lcmtypes_LIB}
                         ${drakeJoints_LIB}
                         ${drakeIKoptions_LIB}
                         ${drakeRigidBodyConstraint_LIB})