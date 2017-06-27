cmake_minimum_required(VERSION 2.8.3)
project(pc_extractor)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  pcl_conversions
  roscpp
  rospy
  sensor_msgs
  visualization_msgs  
  message_generation
  message_filters
  roslib
  std_msgs
  tmc_yolo2_ros
  tf
  cv_bridge
)

#add_service_files(
#   FILES
#)

#generate_messages(
#  DEPENDENCIES
#  sensor_msgs
#  std_msgs
#)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES villa_3d_object_extract
  CATKIN_DEPENDS 
  	roscpp 
  	message_runtime
    tmc_yolo2_ros
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
#add_definitions(${PCL_DEFINITIONS})
#link_directories(${PCL_LIBRARY_DIRS} /usr/local/lib)

include_directories(
    include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(tmc_cluster_based_yolo_3D_extractor src/tmc_cluster_based_yolo_3D_extractor.cpp)
target_link_libraries(tmc_cluster_based_yolo_3D_extractor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(human_tmc_cluster_based_yolo_3D_extractor src/human_tmc_cluster_based_yolo_3D_extractor.cpp)
target_link_libraries(human_tmc_cluster_based_yolo_3D_extractor ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pc_image_repeater_180_flip src/pc_image_repeater_180_flip.cpp)
target_link_libraries(pc_image_repeater_180_flip ${catkin_LIBRARIES} ${PCL_LIBRARIES})
