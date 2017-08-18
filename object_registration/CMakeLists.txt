cmake_minimum_required(VERSION 2.8.3)
project(object_registration)

add_compile_options(-std=c++11)
## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  #pcl_ros
  pcl_conversions
  roscpp
  rospy
  sensor_msgs
  visualization_msgs  
  geometry_msgs
  interactive_markers  
  message_generation
  message_filters
  roslib
  std_msgs
  tf
  cv_bridge
)

find_package(PCL 1.7 REQUIRED)

add_service_files(
   FILES
   ObjectLocalize.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
  std_msgs 
)

catkin_package(
  CATKIN_DEPENDS 
  	roscpp 
  	message_runtime
    cv_bridge
    pcl_ros
    rospy
  	sensor_msgs
  	std_msgs
    geometry_msgs
  DEPENDS
  	roscpp
	system_lib
	tf
)

# This must occur before including catkin package directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Include remaining catkin packages
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#--------------------------------------------------------------
# Package Nodes
#--------------------------------------------------------------
add_executable(interest_box_marker src/interest_box_marker.cpp)
target_link_libraries(interest_box_marker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(interest_box_marker ${catkin_EXPORTED_TARGETS})

add_executable(object_localize_service src/object_localize_service.cpp)
target_link_libraries(object_localize_service ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(object_localize_service ${catkin_EXPORTED_TARGETS})

#--------------------------------------------------------------
# Test Nodes
#--------------------------------------------------------------
add_executable(test_object_localize_service test/test_object_localize_service.cpp)
target_link_libraries(test_object_localize_service ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(test_object_localize_service ${catkin_EXPORTED_TARGETS})

#--------------------------------------------------------------
# Reference Codes
#--------------------------------------------------------------
add_executable(icp_example reference_codes/icp_example.cpp)
target_link_libraries(icp_example ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(object_pose_estimate_sample reference_codes/object_pose_estimate_sample.cpp)
target_link_libraries(object_pose_estimate_sample ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(object_pose_estimate_iss_keypoints_and_icp reference_codes/object_pose_estimate_iss_keypoints_and_icp.cpp)
target_link_libraries(object_pose_estimate_iss_keypoints_and_icp ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(yaml-cpp_test reference_codes/yaml-cpp_test.cpp)
target_link_libraries(yaml-cpp_test ${catkin_LIBRARIES} yaml-cpp)



