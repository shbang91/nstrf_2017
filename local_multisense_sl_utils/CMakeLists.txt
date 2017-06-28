cmake_minimum_required(VERSION 2.8.3)
project(local_multisense_sl_utils)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs  
  pcl_ros
  pcl_conversions
  cv_bridge
  message_filters
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
#  CATKIN_DEPENDS other_catkin_pkg
)

include_directories(
	${PCL_INCLUDE_DIRS}
)

add_executable(pc_image_repeater_180_flip src/pc_image_repeater_180_flip.cpp)
target_link_libraries(pc_image_repeater_180_flip ${catkin_LIBRARIES} ${PCL_LIBRARIES})
