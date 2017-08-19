#include <ros/ros.h>
#include <ros/package.h>


#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>

#include "object_registration/ObjectLocalize.h"
#include "quat_helper.h"
#include "geometry_msgs/Pose.h"

#define PCL_VISUALIZE false

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;


void run_icp_on_pc(const PointCloudT::Ptr& cloud_in, 
                   const PointCloudT::Ptr& cloud_target,
                   const PointCloudT::Ptr& aligned_output,
                   Eigen::Matrix4f& icp_transform_out,
                   double& fitnessScore){ 
  std::cout << "Running ICP" << std::endl;

  std::cout << "Input Cloud Size:" << cloud_in->points.size() << " Target Cloud Size:" << cloud_target->points.size() << std::endl;
  pcl::IterativeClosestPoint<PointNT, PointNT> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_target);
  icp.setUseReciprocalCorrespondences(true);
  PointCloudT Final;
  icp.align(Final);

  (*aligned_output) = Final;

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;

  icp_transform_out = icp.getFinalTransformation();
  fitnessScore = icp.getFitnessScore();

  std::cout << icp_transform_out << std::endl;
}

bool object_localize(const PointCloudT::Ptr& object_in, const PointCloudT::Ptr& scene_in, geometry_msgs::Pose& relative_pose){
  // Point clouds
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr object_aligned (new PointCloudT);
  PointCloudT::Ptr scene (new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);

  // Normals
  PointCloudT::Ptr object_normals (new PointCloudT);  
  PointCloudT::Ptr scene_normals (new PointCloudT);  
   
  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.005f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object_in);
  grid.filter (*object);
  grid.setInputCloud (scene_in);
  grid.filter (*scene);
  
  // Estimate normals for scene
/*  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  nest.setRadiusSearch (0.01);
  nest.setInputCloud (scene);
  nest.compute (*scene);*/


  // Slower Estimation of Normals, but produces better Pose Estimate bfore ICP is run
  pcl::NormalEstimation<PointNT, PointNT> normalEstimation;
  pcl::search::KdTree<PointNT>::Ptr kdtree(new pcl::search::KdTree<PointNT>);
  normalEstimation.setRadiusSearch( 0.01 ); //0.06 .045 0.1 .03 .1 
  normalEstimation.setSearchMethod(kdtree);
  
  normalEstimation.setInputCloud(scene); // only do keypoints  
  normalEstimation.setSearchSurface(scene); // use the entire cloud to define the keypoint's normal
  normalEstimation.compute(*scene_normals);

  normalEstimation.setInputCloud(object); // only do keypoints  
  normalEstimation.setSearchSurface(object); // use the entire cloud to define the keypoint's normal
  normalEstimation.compute(*object_normals);  

  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (0.025);
  fest.setInputCloud (object);
  fest.setInputNormals (object_normals);
//  fest.setInputNormals (object);
  fest.compute (*object_features);
  fest.setInputCloud (scene);
  fest.setInputNormals (scene_normals);
//  fest.setInputNormals (scene);
  fest.compute (*scene_features);
  
  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
  align.setInputSource (object);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (50000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }
  
  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());


  // Extract the inliers
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());    
    std::vector<int> scene_inliers; scene_inliers = align.getInliers();   
    inliers->indices = scene_inliers;

    PointCloudT::Ptr inliers_cloud (new PointCloudT);


    pcl::ExtractIndices<PointNT> extract;
   // Extract the inliers
    extract.setInputCloud (object_aligned);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*inliers_cloud);    


    // Perform ICP and get the 4x4 Transform
    Eigen::Matrix4f icp_transform_out;
    double fitnessScore = 1000;
    PointCloudT::Ptr icp_transformed_cloud (new PointCloudT);
    run_icp_on_pc(object_aligned, scene, icp_transformed_cloud, icp_transform_out, fitnessScore);
    // Execute the Transform for ICP

    //pcl::transformPointCloud (*object_aligned, *icp_transformed_cloud, icp_transform_out);

    PointCloudT::Ptr transform_cloud (new PointCloudT);
    pcl::transformPointCloud (*object_in, *transform_cloud, transformation);





    Eigen::Affine3f affine_transform;
    RotMat3f transform_rot; 
    Vector3f transform_translate; 

    affine_transform.matrix() = transformation; //transformation
    transform_rot = affine_transform.rotation();
    transform_translate = affine_transform.translation();

    pcl::console::print_info("transform -> affine-> rot and linear");
    pcl::console::print_info ("    | %f %f %f | \n", transform_rot (0,0), transform_rot (0,1), transform_rot (0,2));
    pcl::console::print_info ("R = | %f %f %f | \n", transform_rot (1,0), transform_rot (1,1), transform_rot (1,2));
    pcl::console::print_info ("    | %f %f %f | \n", transform_rot (2,0), transform_rot (2,1), transform_rot (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transform_translate (0), transform_translate (1), transform_translate (2));
    pcl::console::print_info ("\n");


    // Fill in answer
    relative_pose.orientation = R_to_quat(transform_rot);
    relative_pose.position.x = transform_translate (0);
    relative_pose.position.y = transform_translate (1); 
    relative_pose.position.z = transform_translate (2);  

    if (PCL_VISUALIZE){
      // Show alignment
      pcl::visualization::PCLVisualizer visu("Alignment");
      visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
      visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
 //     visu.addPointCloud (icp_transformed_cloud, ColorHandlerT (icp_transformed_cloud, 255.0, 0.0, 255.0), "icp_object_aligned");
 //     visu.addPointCloud (inliers_cloud, ColorHandlerT (inliers_cloud, 255.0, 255.0, 0.0), "object_inliers");    
      visu.addPointCloud (transform_cloud, ColorHandlerT (transform_cloud, 255.0, 255.0, 0.0), "transform_cloud");          
      visu.spin ();
    }

    //
    
    // Convert Matrix 4x4 to translation and quaternion:
    // Extract 3x1 translation vector, p
    // Extract 3x3 rotation matrix, R
    // convert R to quat
    // compose and send message




    return true;
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    return true;
  }
  
  return false;
}

// Test
void test_simple_registration(){  
  PointCloudT::Ptr object_in (new PointCloudT);
  PointCloudT::Ptr scene_in (new PointCloudT);
  pcl::console::print_highlight ("Loading point clouds...\n");
  std::string package_path = ros::package::getPath("object_registration");
  std::string filepath = package_path + "/pcd_examples";
  // Load Object Point Cloud File 1
  pcl::io::loadPCDFile (filepath + "/chef.pcd", *object_in);
  // Load Scene Point Cloud File 2
  pcl::io::loadPCDFile (filepath + "/rs1.pcd", *scene_in);   

  geometry_msgs::Pose relative_pose;
  object_localize(object_in, scene_in, relative_pose);
}

bool localize_service(object_registration::ObjectLocalize::Request  &req,
                      object_registration::ObjectLocalize::Response &res){
/*  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);*/

  ROS_INFO("Service requested");
  PointCloudT::Ptr object_in (new PointCloudT);
  PointCloudT::Ptr scene_in (new PointCloudT);

  pcl::fromROSMsg(req.object_cloud, *object_in);   // Perform Copy
  pcl::fromROSMsg(req.target_cloud, *scene_in);   // Perform Copy  

  return  object_localize(object_in, scene_in, res.pose_offset);
}

// Align a rigid object to a scene with clutter and occlusions
int main (int argc, char **argv){
  ros::init(argc, argv, "object_localizer");
  ros::NodeHandle nh;

  // Advertise Service
  ros::ServiceServer service = nh.advertiseService("object_registration/object_localizer_service", localize_service);

  //test_simple_registration();

  // Spin forever
  ros::spin();
  return 0;
}