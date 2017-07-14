cmake_minimum_required(VERSION 2.8.3)
project(val_ik)

set(DRAKE_CATKIN_WS /home/stevenjj/dev/drake_catkin_workspace)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  visualization_msgs  
  message_generation
  message_filters
  roslib
  std_msgs
  val_ik_msgs  
  tf
  cmake_modules
)

add_service_files(
  FILES
  DrakeFKBodyPose.srv
  DrakeIKVal.srv
  DrakeOneHandSingleIk.srv
  DrakeTwoHandsSingleIk.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
  val_ik_msgs
)

find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  	roscpp 
  	message_runtime
    rospy
  	sensor_msgs
    geometry_msgs
  	std_msgs
    val_ik_msgs    
  DEPENDS
  	Eigen
  	roscpp
	system_lib
	tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${DRAKE_CATKIN_WS}/install/include
  ${DRAKE_CATKIN_WS}/install/include/lcmtypes
  ${DRAKE_CATKIN_WS}/install/include/util  

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



add_executable(val_ik_service src/val_ik_service.cpp src/convexHull.cpp src/global_vars.cpp)
target_link_libraries(val_ik_service ${catkin_LIBRARIES} ${DRAKECOMMON_LIB} 
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
add_dependencies(val_ik_service ${catkin_EXPORTED_TARGETS})

add_executable(se3_traj src/se3_traj.cpp)
target_link_libraries(se3_traj ${catkin_LIBRARIES})
add_dependencies(se3_traj ${catkin_EXPORTED_TARGETS})


# Everything below are test nodes
add_executable(val_ik src/test/val_ik.cpp)
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
add_dependencies(val_ik ${catkin_EXPORTED_TARGETS})                         

add_executable(val_ik_ros_pub_test src/test/val_ik_ros_pub_test.cpp)
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
add_dependencies(val_ik_ros_pub_test ${catkin_EXPORTED_TARGETS})


add_executable(val_ik_ros_srv_node src/test/val_ik_ros_srv_node.cpp)
target_link_libraries(val_ik_ros_srv_node ${catkin_LIBRARIES} ${DRAKECOMMON_LIB} 
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
add_dependencies(val_ik_ros_srv_node ${catkin_EXPORTED_TARGETS})

add_executable(test_ik_service src/test/test_ik_service.cpp)
target_link_libraries(test_ik_service ${catkin_LIBRARIES})
add_dependencies(test_ik_service ${catkin_EXPORTED_TARGETS})

add_executable(test_ik_simple_service src/test/test_ik_simple_service.cpp)
target_link_libraries(test_ik_simple_service ${catkin_LIBRARIES})
add_dependencies(test_ik_simple_service ${catkin_EXPORTED_TARGETS})

add_executable(val_single_ik_srv src/val_single_ik_srv.cpp src/global_vars.cpp)
target_link_libraries(val_single_ik_srv ${catkin_LIBRARIES})
add_dependencies(val_single_ik_srv ${catkin_EXPORTED_TARGETS})


