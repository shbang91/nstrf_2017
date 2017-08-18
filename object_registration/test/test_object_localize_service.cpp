#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

#include "object_registration/ObjectLocalize.h"

typedef pcl::PointXYZRGB PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;


ros::ServiceClient      object_localize_client;

sensor_msgs::PointCloud2 object_cloud_msg;
sensor_msgs::PointCloud2 scene_cloud_msg;    



bool call_localize_object_service(PointCloudT::Ptr object_in, PointCloudT::Ptr scene_in){
    object_registration::ObjectLocalize srv;

    sensor_msgs::PointCloud2 object_msg;
    sensor_msgs::PointCloud2 scene_msg;    

    pcl::toROSMsg(*object_in, object_msg);
    pcl::toROSMsg(*scene_in, scene_msg);    
/*    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);*/

    ROS_INFO("Calling service");
    if (object_localize_client.call(srv)){
        ROS_INFO("Service returned");
        return true;
    }
    else{
        ROS_ERROR("Failed to call service object_registration/object_localizer_service");
        return false;
    }    

}

void test_service_call(){
    PointCloudT::Ptr object_in (new PointCloudT);
    PointCloudT::Ptr scene_in (new PointCloudT);
    pcl::console::print_highlight ("Loading point clouds...\n");
    std::string package_path = ros::package::getPath("object_registration");
    std::string filepath = package_path + "/pcd_examples";
    // Load Object Point Cloud File 1
    pcl::io::loadPCDFile (filepath + "/chef.pcd", *object_in);
    // Load Scene Point Cloud File 2
    pcl::io::loadPCDFile (filepath + "/rs1.pcd", *scene_in);   

    std::cout << object_in->points.size() << std::endl;
    std::cout << scene_in->points.size() << std::endl;  

    pcl::toROSMsg(*object_in, object_cloud_msg);
    pcl::toROSMsg(*scene_in, scene_cloud_msg);    

    object_cloud_msg.header.frame_id = "world";
    scene_cloud_msg.header.frame_id = "world";    

    call_localize_object_service(object_in, scene_in);

//  pcl::fromROSMsg(*msg, *cloud);   // Perform Copy
}

int main (int argc, char **argv){
    ros::init(argc, argv, "test_object_localizer");
    ros::NodeHandle         nh;
    
    ros::Publisher object_cloud_pub;
    ros::Publisher scene_cloud_pub;

    object_localize_client = nh.serviceClient<object_registration::ObjectLocalize>("object_registration/object_localizer_service");

    object_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("object_in", 0);
    scene_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("scene_in", 0);

    test_service_call();
    ros::Rate r(20);
    while(true){
        object_cloud_pub.publish(object_cloud_msg);
        scene_cloud_pub.publish(scene_cloud_msg);   
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}