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
  interactive_markers  
  message_generation
  message_filters
  roslib
  std_msgs
  tf
  cv_bridge
)

find_package(PCL 1.7 REQUIRED)

#add_service_files(
#   FILES
#)

#generate_messages(
#  DEPENDENCIES
#  sensor_msgs
#  std_msgs 
#)

catkin_package(
  CATKIN_DEPENDS 
  	roscpp 
  	message_runtime
    cv_bridge
    pcl_ros
    rospy
  	sensor_msgs
  	std_msgs
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

add_executable(icp_example src/icp_example.cpp)
target_link_libraries(icp_example ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(object_pose_estimate_sample src/object_pose_estimate_sample.cpp)
target_link_libraries(object_pose_estimate_sample ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(object_pose_estimate_iss_keypoints_and_icp src/object_pose_estimate_iss_keypoints_and_icp.cpp)
target_link_libraries(object_pose_estimate_iss_keypoints_and_icp ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(basic_controls src/basic_controls.cpp)
target_link_libraries(basic_controls ${catkin_LIBRARIES})


add_executable(interest_box_marker src/interest_box_marker.cpp)
target_link_libraries(interest_box_marker ${catkin_LIBRARIES})



