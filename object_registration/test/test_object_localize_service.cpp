#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Core>
#include "object_registration/ObjectLocalize.h"
#include "quat_helper.h"
#include "geometry_msgs/Pose.h"

typedef pcl::PointXYZRGB PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;


ros::ServiceClient      object_localize_client;

sensor_msgs::PointCloud2 object_cloud_msg;
sensor_msgs::PointCloud2 scene_cloud_msg;
sensor_msgs::PointCloud2 object_transformed_cloud_msg;    



bool call_localize_object_service(PointCloudT::Ptr object_in, PointCloudT::Ptr scene_in){
    object_registration::ObjectLocalize srv;

    sensor_msgs::PointCloud2 object_msg;
    sensor_msgs::PointCloud2 scene_msg;    

    pcl::toROSMsg(*object_in, object_msg);
    pcl::toROSMsg(*scene_in, scene_msg);    
    srv.request.object_cloud = object_msg;
    srv.request.target_cloud = scene_msg;


    ROS_INFO("Calling service");
    if (object_localize_client.call(srv)){
        ROS_INFO("Service returned");

        std::cout << "Position (x,y,z):" << srv.response.pose_offset.position.x << " "
                                         << srv.response.pose_offset.position.y << " "
                                         << srv.response.pose_offset.position.z << " "
                                         << std::endl;
        std::cout << "Quat (x,y,z,w):" << srv.response.pose_offset.orientation.x << " "
                                       << srv.response.pose_offset.orientation.y << " "
                                       << srv.response.pose_offset.orientation.z << " "
                                       << srv.response.pose_offset.orientation.w 
                                       << std::endl;

        Vector3f pose_translation(srv.response.pose_offset.position.x,  srv.response.pose_offset.position.y,  srv.response.pose_offset.position.z);
        RotMat3f pose_rotation;                                                


        pose_rotation = quat_to_R(srv.response.pose_offset.orientation);       

        ROS_INFO ("    | %6.3f %6.3f %6.3f | ", pose_rotation (0,0), pose_rotation (0,1), pose_rotation (0,2));
        ROS_INFO ("R = | %6.3f %6.3f %6.3f | ", pose_rotation (1,0), pose_rotation (1,1), pose_rotation (1,2));
        ROS_INFO ("    | %6.3f %6.3f %6.3f | ", pose_rotation (2,0), pose_rotation (2,1), pose_rotation (2,2));
        ROS_INFO ("t = < %0.3f, %0.3f, %0.3f >\n", pose_translation (0), pose_translation (1), pose_translation (2));

        Eigen::Matrix4f SE3_transform;
        SE3_transform.block<3,3>(0,0) = pose_rotation;
        SE3_transform.block<3,1>(0,3) = pose_translation;        
        SE3_transform (3,0) = 0.0;  SE3_transform (3,1) = 0.0; SE3_transform (3,2) = 0.0; SE3_transform (3,3) = 1.0;         

        ROS_INFO ("    | %f %f %f | ", SE3_transform (0,0), SE3_transform (0,1), SE3_transform (0,2));
        ROS_INFO ("R = | %f %f %f | ", SE3_transform (1,0), SE3_transform (1,1), SE3_transform (1,2));
        ROS_INFO ("    | %f %f %f | ", SE3_transform (2,0), SE3_transform (2,1), SE3_transform (2,2));
        ROS_INFO ("t = < %0.3f, %0.3f, %0.3f >", SE3_transform (0,3), SE3_transform (1,3), SE3_transform (2,3));
        ROS_INFO ("Last Row: < %0.7f, %0.7f, %0.7f,%0.7f  >", SE3_transform (3,0), SE3_transform (3,1), SE3_transform (3,2), SE3_transform (3,3) );


        PointCloudT::Ptr object_transformed (new PointCloudT);    
        pcl::transformPointCloud (*object_in, *object_transformed, SE3_transform);
        pcl::toROSMsg(*object_transformed, object_transformed_cloud_msg);
        object_transformed_cloud_msg.header.frame_id = "world"; 
        // convert position to translation
        // convert quaternion to 3x3 rotation matrix

        // construct eigen matrix 4x4 again
        // transform pointcloud to visualize


        // create tf::Stamped Transform 
        // transform.setRotation()
        // transform.setOrigin()


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


}

int main (int argc, char **argv){
    ros::init(argc, argv, "test_object_localizer");
    ros::NodeHandle         nh;
    
    ros::Publisher object_cloud_pub;
    ros::Publisher scene_cloud_pub;
    ros::Publisher object_trans_cloud_pub;

    object_localize_client = nh.serviceClient<object_registration::ObjectLocalize>("object_registration/object_localizer_service");

    object_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("object_in", 0);
    scene_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("scene_in", 0);
    object_trans_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("object_trans", 0);    

    test_service_call();
    ros::Rate r(20);
    while(true){
        object_cloud_pub.publish(object_cloud_msg);
        scene_cloud_pub.publish(scene_cloud_msg);   
        object_trans_cloud_pub.publish(object_transformed_cloud_msg);
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}