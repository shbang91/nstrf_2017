// This code rotates the image by 180 degrees and reverses the order of the pointcloud.
//      The object detection module is sensitive to the rotation of the camera as it expects objects to be right side up
//      Reversing the order of the pointcloud is necessary for corresponding the 2D bounding box to the 3D points

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <cv_bridge/cv_bridge.h>

#include <openpose_ros_msgs/GetPersons.h>
// Declare Publishers
ros::Publisher           flipped_pc_order_pub;
ros::Publisher           flipped_image_pub;   

// Declare Service Client
ros::ServiceClient client;
openpose_ros_msgs::GetPersons srv;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

//Declare Callback
void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::ImageConstPtr& image_msg){
    ROS_INFO("Cloud and Image Messages Received!");
    ROS_INFO("    Cloud Time Stamp: %f", cloud_msg->header.stamp.toSec());
    ROS_INFO("    Image Time Stamp: %f", image_msg->header.stamp.toSec());  

    srv.request.image = *image_msg;
      if (client.call(srv))
      {
        ROS_INFO("HELLO!");
//        ROS_INFO("Hello!: %ld", (long int)srv.response.sum);
      }
      else
      {
        ROS_ERROR("Failed to call service detect_poses");
      }


}

int main(int argc, char** argv){
    // Initialize ROS
    ros::init(argc, argv, "test_openpose_ros_service_call");

    // Declare Node Handle
    ros::NodeHandle nh;

    // Declare Subscribers
    // Synchronize Point Cloud and Image Subscription Received Messages
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/multisense/organized_image_points2_color_reverse_order", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/multisense/left/image_rect_color_rotated_180", 1);

    client = nh.serviceClient<openpose_ros_msgs::GetPersons>("detect_poses");

    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, image_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Advertise Flipped Images and Pointcloud
    //flipped_pc_order_pub = nh.advertise<sensor_msgs::PointCloud2>( "/multisense/organized_image_points2_color_reverse_order", 0 );
    //flipped_image_pub = nh.advertise<sensor_msgs::Image>( "/multisense/left/image_rect_color_rotated_180", 0 );  

    // Spin Forever
    ros::spin();

    return 0;
}
    