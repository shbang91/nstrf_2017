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

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/keypoints/iss_3d.h>
#include <pcl/kdtree/kdtree.h>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;



// This function by Tommaso Cavallari and Federico Tombari, taken from the tutorial
// http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
double computeCloudResolution(const PointCloudT::ConstPtr& cloud)
{
  double resolution = 0.0;
  int numberOfPoints = 0;
  int nres;
  std::vector<int> indices(2);
  std::vector<float> squaredDistances(2);
  pcl::search::KdTree<PointNT> tree;
  tree.setInputCloud(cloud);
  
  for (size_t i = 0; i < cloud->size(); ++i)
  {
    if (! pcl_isfinite((*cloud)[i].x))
      continue;
  
    // Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
    if (nres == 2)
    {
      resolution += sqrt(squaredDistances[1]);
      ++numberOfPoints;
    }
  }
  if (numberOfPoints != 0)
    resolution /= numberOfPoints;
  
  return resolution;
}
 
void iss_detector(const PointCloudT::Ptr& cloud, PointCloudT::Ptr& keypoints_out ){
      // ISS keypoint detector object.
      pcl::ISSKeypoint3D<PointNT, PointNT> detector;
      detector.setInputCloud(cloud);
      pcl::search::KdTree<PointNT>::Ptr kdtree(new pcl::search::KdTree<PointNT>);
      detector.setSearchMethod(kdtree);
      double resolution = computeCloudResolution(cloud);
      // Set the radius of the spherical neighborhood used to compute the scatter matrix.
      detector.setSalientRadius(6 * resolution);
      // Set the radius for the application of the non maxima supression algorithm.
      detector.setNonMaxRadius(4 * resolution);
      // Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
      detector.setMinNeighbors(5);
      // Set the upper bound on the ratio between the second and the first eigenvalue.
      detector.setThreshold21(0.975);
      // Set the upper bound on the ratio between the third and the second eigenvalue.
      detector.setThreshold32(0.975);
      // Set the number of prpcessing threads to use. 0 sets it to automatic.
      detector.setNumberOfThreads(4);
      
      detector.compute(*keypoints_out);
 
}


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

// Align a rigid object to a scene with clutter and occlusions
int
main (int argc, char **argv)
{
  // Point clouds
  PointCloudT::Ptr object_keypoints (new PointCloudT);
  PointCloudT::Ptr scene_keypoints (new PointCloudT);


  PointCloudT::Ptr object_normals (new PointCloudT);  
  PointCloudT::Ptr scene_normals (new PointCloudT);  

  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr object_aligned (new PointCloudT);
  PointCloudT::Ptr scene (new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);
  
  // Get input object and scene
/*  if (argc != 3)
  {
    pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
    return (1);
  }*/
  
  // Load object and scene
  pcl::console::print_highlight ("Loading point clouds...\n");

  std::string package_path = ros::package::getPath("object_registration");
  std::string filepath = package_path + "/pcd_examples";
  // Load Object Point Cloud File 1
  pcl::io::loadPCDFile (filepath + "/chef.pcd", *object);
  // Load Scene Point Cloud File 2
  pcl::io::loadPCDFile (filepath + "/rs1.pcd", *scene);   

  
  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.005f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  grid.filter (*object);
  grid.setInputCloud (scene);
  grid.filter (*scene);

  // Calculate ISS Keypoints
  pcl::console::print_highlight ("Detecting ISS Keypoints...\n");
  //iss_detector(object, object_keypoints);
  //iss_detector(scene, scene_keypoints);  
  (*scene_keypoints) = *scene;
  (*object_keypoints) = *object;  

  
  // Estimate normals for scene
/*  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  nest.setRadiusSearch (0.01);
  nest.setInputCloud (scene);
  nest.compute (*scene_normals);
  
  nest.setInputCloud (object);
  nest.compute (*object_normals);
*/
  pcl::NormalEstimation<PointNT, PointNT> normalEstimation;
  pcl::search::KdTree<PointNT>::Ptr kdtree(new pcl::search::KdTree<PointNT>);
  normalEstimation.setRadiusSearch( 0.01 ); //0.06 .045 0.1 .03 .1 
  normalEstimation.setSearchMethod(kdtree);

  normalEstimation.setInputCloud(scene_keypoints); // only do keypoints  
  normalEstimation.setSearchSurface(scene); // use the entire cloud to define the keypoint's normal
  normalEstimation.compute(*scene_normals);

  normalEstimation.setInputCloud(object_keypoints); // only do keypoints  
  normalEstimation.setSearchSurface(object); // use the entire cloud to define the keypoint's normal
  normalEstimation.compute(*object_normals);  


  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (0.025);
  fest.setInputCloud (object_keypoints); //fest.setInputCloud (object);
  fest.setInputNormals (object_normals);
  fest.compute (*object_features);
  fest.setInputCloud (scene_keypoints);//fest.setInputCloud (scene);
  fest.setInputNormals (scene_normals);
  fest.compute (*scene_features);
  
  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
  align.setInputSource (object_keypoints);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene_keypoints);
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
    run_icp_on_pc(object_aligned, scene_keypoints, icp_transformed_cloud, icp_transform_out, fitnessScore);
    // Execute the Transform for ICP

    //pcl::transformPointCloud (*object_aligned, *icp_transformed_cloud, icp_transform_out);

    // Show alignment
    pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
    visu.addPointCloud (icp_transformed_cloud, ColorHandlerT (icp_transformed_cloud, 255.0, 0.0, 255.0), "icp_object_aligned");
    visu.addPointCloud (inliers_cloud, ColorHandlerT (inliers_cloud, 255.0, 255.0, 0.0), "object_inliers");    



/*
    // Perform ICP on the ISS keypoints of the aligned object
    PointCloudT::Ptr object_post_aligned_iss_keypoints (new PointCloudT);
    PointCloudT::Ptr scene_iss_keypoints (new PointCloudT);
    iss_detector(object_aligned, object_post_aligned_iss_keypoints);
    iss_detector(scene, scene_iss_keypoints); 

    Eigen::Matrix4f icp_transform_out_post;
    double fitnessScore_post = 1000;
    PointCloudT::Ptr icp_transformed_cloud_post (new PointCloudT);   
    run_icp_on_pc(object_post_aligned_iss_keypoints, scene_iss_keypoints, icp_transformed_cloud_post, icp_transform_out_post, fitnessScore_post);

   // Execute the Transform
    pcl::PointCloud<PointNT>::Ptr iss_transform_aligned_cloud (new pcl::PointCloud<PointNT>);
    pcl::transformPointCloud (*object_aligned, *iss_transform_aligned_cloud, icp_transform_out_post);
   
    visu.addPointCloud (icp_transformed_cloud_post, ColorHandlerT (icp_transformed_cloud_post, 255.0, 0.0, 0.0), "icp_transformed_cloud_post");    
    visu.addPointCloud (iss_transform_aligned_cloud, ColorHandlerT (iss_transform_aligned_cloud, 255.0, 255.0, 0.0), "iss_transform_aligned_cloud");    

*/

    visu.spin ();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    //visu.addPointCloud (scene_keypoints, ColorHandlerT (scene_keypoints, 255.0, 0.0, 0.0), "scene_keypoints");    

    visu.spin ();     
//    return (1);
  }

  

  return (0);
}