/*
Simple Reconstruction Using ICP from consecutive pcd files

Data:
  1. Global model
  2. Current Frame

1. Input a new pcd -- new_cloud
2. Preprocessing
  1) Fast Bilateral Filter
  2) Voxel Grid -- new_cloud_voxel
3. Match to previous frame - trans_mat
4. Translate the frame to Global model
5. Combine to Global model
*/

#include <iostream>
#include <random>
#include <sstream>
#include <string>
#define VIZ

#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>

#ifdef VIZ
  #include <pcl/visualization/cloud_viewer.h>
  #include <pcl/visualization/pcl_visualizer.h>
#endif
 
int main (int argc, char** arg) {
  // Global Model
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr global_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointNormal>::Ptr  global_cloud_with_normal (new pcl::PointCloud<pcl::PointNormal> ());
  Eigen::Matrix4f accumulated_trans = Eigen::Matrix4f::Identity (); 
  
  // Initialze Bilateral Filter
  float sigma_s = 10.f, sigma_r = 0.5f;
  pcl::FastBilateralFilter<pcl::PointXYZRGBA> bilateral_filter;
  bilateral_filter.setSigmaS (sigma_s);
  bilateral_filter.setSigmaR (sigma_r);
  // Initialize Voxel Grid
  pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;
  voxel_grid.setLeafSize (0.05, 0.05, 0.05);
  // Initialize Normal Estimation
  pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::PointNormal> norm_est;
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
  norm_est.setSearchMethod (kdtree);
  norm_est.setKSearch (30);

  // Initialize ICP
  pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp;
  //icp.setTransformationEpsilon (1e-6);
  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  // icp.setMaxCorrespondenceDistance (0.05);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (100);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon (1e-10);
  // Set the euclidean distance difference epsilon (criterion 3)
  //icp.setEuclideanFitnessEpsilon (1);

#ifdef VIZ
  pcl::visualization::PCLVisualizer *viewer = new pcl::visualization::PCLVisualizer ("Registration");
#endif

  int frame_num = 150;
  for (int frame_id = 1; frame_id <= frame_num; ++frame_id) {
    // Get new pcd name
    std::stringstream new_pcd_file_name;
    new_pcd_file_name << "../data/my_room_seq/" << frame_id << ".pcd";
    std::cout << "Reading file " << new_pcd_file_name.str() << std::endl;
    // Get new point cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile(new_pcd_file_name.str(), *new_cloud);

    // Do Fast Bilateral Filter
    bilateral_filter.setInputCloud (new_cloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
    bilateral_filter.filter(*new_cloud_filtered);
    std::cout << "Done bilateral filter.\n";

    // Do Voxel Grid
    voxel_grid.setInputCloud (new_cloud_filtered);
    voxel_grid.filter(*new_cloud_filtered);
    std::cout << "Done Voxel Grid.\n";

    if (frame_id == 1) {
      //  If first frame then it is the Global Model
      *global_cloud = *new_cloud_filtered;
      norm_est.setInputCloud (global_cloud);
      norm_est.compute (*global_cloud_with_normal);
      pcl::copyPointCloud (*global_cloud, *global_cloud_with_normal);
      std::cout << "First frame added as initial global model\n";
#ifdef VIZ
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(global_cloud);
      viewer->addPointCloud<pcl::PointXYZRGBA> (global_cloud, rgb, "global model");
      // viewer->spin();
#endif
    } else {
      // Match the frame to global model 
      // TODO: refine maximum correspondence distance each loop in ICP

      // Setup new cloud
      pcl::PointCloud<pcl::PointNormal>::Ptr  new_cloud_with_normal (new pcl::PointCloud<pcl::PointNormal> ());
      norm_est.setInputCloud (new_cloud_filtered);
      norm_est.compute (*new_cloud_with_normal);
      pcl::copyPointCloud (*new_cloud_filtered, *new_cloud_with_normal);      

      // Do Non-linear ICP
      icp.setInputSource (new_cloud_with_normal);
      icp.setInputTarget (global_cloud_with_normal);
      pcl::PointCloud<pcl::PointNormal> Final;
      icp.align(Final);

      // Print result data
      std::cout << "has converged:" << icp.hasConverged() << " score: " <<
      icp.getFitnessScore() << std::endl;
      std::cout << icp.getFinalTransformation() << std::endl;

      // Merge cloud to global data
      //accumulated_trans = icp.getFinalTransformation () * accumulated_trans;
      accumulated_trans = accumulated_trans * icp.getFinalTransformation ();

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::transformPointCloud (*new_cloud_filtered, *output, accumulated_trans);
      
      *global_cloud += *output;

      // Do Voxel Grid
      voxel_grid.setInputCloud (global_cloud);
      voxel_grid.filter(*global_cloud);
      std::cout << "Done Voxel Grid on Global Model.\n";
      norm_est.setInputCloud (global_cloud);
      norm_est.compute (*global_cloud_with_normal);
      pcl::copyPointCloud (*global_cloud, *global_cloud_with_normal);


#ifdef VIZ
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(global_cloud);
      viewer->removePointCloud("global model");
      viewer->addPointCloud<pcl::PointXYZRGBA> (global_cloud, rgb, "global model");
      viewer->spin();
#endif
    }    
  }
  viewer->spin();

  return 0;
}
