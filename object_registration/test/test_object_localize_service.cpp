#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

int main (int argc, char **argv){
  ros::init(argc, argv, "test_object_localizer");
 
  return 0;
}