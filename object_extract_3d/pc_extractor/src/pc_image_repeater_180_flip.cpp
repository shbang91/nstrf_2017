#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

//Declare Callback
void callback(const sensor_msgs::PointCloud2ConstPtr& cloud, const sensor_msgs::ImageConstPtr& image){
	ROS_INFO("Cloud and Image Messages Received!");
	ROS_INFO("    Cloud Time Stamp: %f", cloud->header.stamp.toSec());
	ROS_INFO("    Image Time Stamp: %f", image->header.stamp.toSec());	

}

int main(int argc, char** argv){
  // Initialize ROS
  ros::init(argc, argv, "multisense_cloud_image_repeater_node");

  // Declare Node Handle
  ros::NodeHandle nh;

  // Synchronize Point Cloud and Image Subscription Received Messages
  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/multisense/image_points2_color", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/multisense/left/image_rect_color", 1);

  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, image_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  // Spin Forever
  ros::spin();

  return 0;
}