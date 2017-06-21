cmake_minimum_required(VERSION 2.8.3)
project(grabcut)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS  
  cv_bridge 
  opencv2
)

