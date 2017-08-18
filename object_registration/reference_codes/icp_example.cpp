#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/io/pcd_io.h>
//#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
 #include <pcl/visualization/cloud_viewer.h>


#include <pcl/common/transforms.h>

/*#include <pcl/registration/transformation_estimation_svd.h>*/


void icp_example(){
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the CloudIn data
  cloud_in->width    = 5;
  cloud_in->height   = 1;
  cloud_in->is_dense = false;
  cloud_in->points.resize (cloud_in->width * cloud_in->height);
  for (size_t i = 0; i < cloud_in->points.size (); ++i)
  {
    cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
      << std::endl;

  for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
      cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<   cloud_in->points[i].z << std::endl;
  *cloud_out = *cloud_in;
  std::cout << "size:" << cloud_out->points.size() << std::endl;


  for (size_t i = 0; i < cloud_in->points.size (); ++i)
    cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
  std::cout << "Transformed " << cloud_in->points.size () << " data points:"
      << std::endl;
  for (size_t i = 0; i < cloud_out->points.size (); ++i)
    std::cout << "    " << cloud_out->points[i].x << " " <<
      cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
}

void run_icp_on_pc(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, 
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_target,
                   Eigen::Matrix4f& icp_transform_out,
                   double& fitnessScore){ 
  std::cout << "Running ICP" << std::endl;

  std::cout << "Input Cloud Size:" << cloud_in->points.size() << " Target Cloud Size:" << cloud_target->points.size() << std::endl;
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_target);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;

  icp_transform_out = icp.getFinalTransformation();
  fitnessScore = icp.getFitnessScore();

  std::cout << icp_transform_out << std::endl;
}


/*
void svd_est_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr& source, pcl::PointCloud<pcl::PointXYZ>::Ptr& target){
  //pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>(cloud_source));
  //pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>(cloud_target));

  // re-do reciprocal correspondence estimation
  boost::shared_ptr<pcl::Correspondences> correspondences (new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputCloud (source);
  corr_est.setInputTarget (target);
  corr_est.determineReciprocalCorrespondences (*correspondences);

  Eigen::Matrix4f transform_res_from_SVD;
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est_svd;
  trans_est_svd.estimateRigidTransformation(*source, *target,
                                            *correspondences,
  transform_res_from_SVD);

  Eigen::Matrix4f transform_res_from_SVD;
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est_svd;
  trans_est_svd.estimateRigidTransformation(*source, *target, transform_res_from_SVD);	
}*/

int main(int argc, char** argv){
    // Test Basic ICP
    //icp_example();

    // Initialize ROS
    ros::init(argc, argv, "object_registration_node_test");

    // Declare Node Handle
    ros::NodeHandle nh;

    // Define Source and Target Clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);


    std::string package_path = ros::package::getPath("object_registration");
    std::string filepath = package_path + "/pcd_examples";
    // Load Point Cloud File 1
    pcl::io::loadPCDFile (filepath + "/bun0.pcd", *cloud_source);
    // Load Point Cloud File 2
    pcl::io::loadPCDFile (filepath + "/bun4.pcd", *cloud_target);    


    // Perform ICP and get the 4x4 Transform
    Eigen::Matrix4f icp_transform_out;
    double fitnessScore = 1000;
    run_icp_on_pc(cloud_source, cloud_target, icp_transform_out, fitnessScore);

   // Execute the Transform
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud (*cloud_source, *transformed_cloud, icp_transform_out);

    // View Inputs and Outputs
    pcl::visualization::PCLVisualizer viewer("Input Cloud Viewer");

   // Define R,G,B colors for the point clouds
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud_source, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color_handler (cloud_target, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 0, 255, 0);

    viewer.addPointCloud (cloud_source, source_cloud_color_handler, "original_cloud");
    viewer.addPointCloud (cloud_target, target_cloud_color_handler, "target_cloud");
    //viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

    // Save Transformed Cloud
    pcl::io::savePCDFileASCII (package_path + "/pcd_saved_files/icp_example_transformed.pcd", *transformed_cloud);
    std::cerr << "Saved " << transformed_cloud->points.size () << " data points to icp_example_transformed.pcd." << std::endl;

    // Load and Visualize the saved Transformed Cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr saved_cloud (new pcl::PointCloud<pcl::PointXYZ>);    
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> saved_cloud_color_handler (saved_cloud, 0, 255, 0);
    pcl::io::loadPCDFile (package_path + "/pcd_saved_files/icp_example_transformed.pcd", *saved_cloud);        
    viewer.addPointCloud (saved_cloud, saved_cloud_color_handler, "saved_cloud");


    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
      viewer.spinOnce ();
    }
    ROS_INFO("Viewer was Stopped. Node will spin forever.");

    // Spin Forever
    ros::spin();

    return 0;
}
    