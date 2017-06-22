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

    // Convert Image Message to CV
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);

    // Flip Image ----------------------------------------------------------
    // By Copying
    /*    // Copy Image and Rotate it
    cv::Mat matImage = cv_ptr->image;
    cv::Mat matRotated = matImage.clone();
    // Flip Image
    rot90(matRotated,3); //Rotate it by 180 Degrees

    //set rotated image as the new image
    cv_ptr->image = matRotated; */

    // By Mutating
    rot90(cv_ptr->image, 3); //Rotate it by 180 Degrees    
    // End Flip Image -------------------------------------------------------

    // Convert CV image back to ROS message
    sensor_msgs::Image ros_image;
    ros_image = *(cv_ptr->toImageMsg());

    // Publish Image
    flipped_image_pub.publish( ros_image  );

}

int main(int argc, char** argv){
    // Initialize ROS
    ros::init(argc, argv, "multisense_cloud_image_repeater_node");

    // Declare Node Handle
    ros::NodeHandle nh;

    // Declare Subscribers
    // Synchronize Point Cloud and Image Subscription Received Messages
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/multisense/image_points2_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/multisense/left/image_rect_color", 1);

    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, image_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));


    // Advertise Flipped Images and Pointcloud
    flipped_pc_order_pub = nh.advertise<sensor_msgs::PointCloud2>( "/flipped_cloud_order", 0 );
    flipped_image_pub = nh.advertise<sensor_msgs::Image>( "/multisense/left/image_rect_color_rotated_180", 0 );  

    // Spin Forever
    ros::spin();

    return 0;
    }