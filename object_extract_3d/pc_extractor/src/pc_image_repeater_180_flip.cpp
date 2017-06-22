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

// Declare Publishers
ros::Publisher           flipped_pc_order_pub;
ros::Publisher           flipped_image_pub;   

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> MySyncPolicy;

// Rotate Image by 90, -90, or 180 degrees.
// From https://stackoverflow.com/questions/15043152/rotate-opencv-matrix-by-90-180-270-degrees
void rot90(cv::Mat &matImage, int rotflag){
  //1=CW, 2=CCW, 3=180
  if (rotflag == 1){
    cv::transpose(matImage, matImage);  
    cv::flip(matImage, matImage,1); //transpose+flip(1)=CW
  } else if (rotflag == 2) {
    cv::transpose(matImage, matImage);  
    cv::flip(matImage, matImage,0); //transpose+flip(0)=CCW     
  } else if (rotflag ==3){
    cv::flip(matImage, matImage,-1);    //flip(-1)=180          
  } else if (rotflag != 0){ //if not 0,1,2,3:
    std::cout  << "Unknown rotation flag(" << rotflag << ")" << std::endl;
  }
}


//Declare Callback
void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::ImageConstPtr& image_msg){
    ROS_INFO("Cloud and Image Messages Received!");
    ROS_INFO("    Cloud Time Stamp: %f", cloud_msg->header.stamp.toSec());
    ROS_INFO("    Image Time Stamp: %f", image_msg->header.stamp.toSec());  

    // Reverse the order of the pointcloud points --------------------------------------------------------------------------------
    // Process PointCloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr orig_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Convert ROS MSG to pointcloud
    pcl::fromROSMsg(*cloud_msg, *orig_cloud);

    // Create a copy with points in the reverse order
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_reverse_points_order (new pcl::PointCloud<pcl::PointXYZRGB>);    
    // Set the same headers as original point cloud for simplicity   
    cloud_reverse_points_order->header = (*orig_cloud).header;

    // Copy points and push them back in reverse order in reverse order
    size_t total_num_points = orig_cloud->points.size();
    for(size_t i = 1; i < total_num_points + 1; i++){
        cloud_reverse_points_order->points.push_back( orig_cloud->points[total_num_points - i] );
    }
    // Debug Statements
    /*
    ROS_INFO("Original Cloud Points: %zu", total_num_points);
    ROS_INFO("    Last Point  (x,y,z): %f, %f, %f ", orig_cloud->points[0].x, orig_cloud->points[0].y, orig_cloud->points[0].z);
    ROS_INFO("    First Point (x,y,z): %f, %f, %f ", orig_cloud->points[total_num_points-1].x, orig_cloud->points[total_num_points-1].y, orig_cloud->points[total_num_points-1].z);    
    */
    ROS_INFO("Reversed Order Cloud Points: %zu", cloud_reverse_points_order->points.size());    
    /*
    ROS_INFO("    Last Point  (x,y,z): %f, %f, %f ", cloud_reverse_points_order->points[0].x, cloud_reverse_points_order->points[0].y, cloud_reverse_points_order->points[0].z);
    ROS_INFO("    First Point (x,y,z): %f, %f, %f ", cloud_reverse_points_order->points[total_num_points-1].x, cloud_reverse_points_order->points[total_num_points-1].y, cloud_reverse_points_order->points[total_num_points-1].z);        
    */
    // End reversing the order of the pointcloud points -----------------------------------------------------------------------------

    // Process Image
    // Convert Image Message to CV
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);

    // Flip Image ----------------------------------------------------------
    // By Copying
    /*    // Copy Image and Rotate it
    cv::Mat matImage = cv_ptr->image;
    cv::Mat matRotated = matImage.clone();
    rot90(matRotated,3); //Rotate it by 180 Degrees
    //set rotated image as the new image
    cv_ptr->image = matRotated; */

    // By Mutating
    rot90(cv_ptr->image, 3); //Rotate it by 180 Degrees    
    // End Flip Image -------------------------------------------------------

    // Convert pointcloud back to pcl message:
    sensor_msgs::PointCloud2 ros_pc2;
    pcl::toROSMsg(*cloud_reverse_points_order, ros_pc2);
    
    // Convert CV image back to ROS message
    sensor_msgs::Image ros_image;
    ros_image = *(cv_ptr->toImageMsg());

    // Publish PointCloud
    flipped_pc_order_pub.publish( ros_pc2 );

    // Publish Image
    flipped_image_pub.publish( ros_image );

}

int main(int argc, char** argv){
    // Initialize ROS
    ros::init(argc, argv, "multisense_cloud_image_repeater_node");

    // Declare Node Handle
    ros::NodeHandle nh;

    // Declare Subscribers
    // Synchronize Point Cloud and Image Subscription Received Messages
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/multisense/organized_image_points2_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/multisense/left/image_rect_color", 1);

    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, image_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Advertise Flipped Images and Pointcloud
    flipped_pc_order_pub = nh.advertise<sensor_msgs::PointCloud2>( "/multisense/organized_image_points2_color_reverse_order", 0 );
    flipped_image_pub = nh.advertise<sensor_msgs::Image>( "/multisense/left/image_rect_color_rotated_180", 0 );  

    // Spin Forever
    ros::spin();

    return 0;
    }