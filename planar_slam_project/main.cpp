#include <iostream>
#include <random>

#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
   
int user_data;
    
int 
main (int argc, char** arg)
{
    // Load data
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_origin (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile("../data/my_room/1.pcd", *cloud_origin);
    
    //pcl::visualization::CloudViewer viewer("Cloud Viewer");
    
    float sigma_s = 10.f, sigma_r = 0.5f;
    pcl::FastBilateralFilter<pcl::PointXYZRGBA> bilateral_filter;
    bilateral_filter.setInputCloud (cloud_origin);
//    bilateral_filter.setHalfSize (sigma_s);
//    bilateral_filter.setStdDev (sigma_r);
    bilateral_filter.setSigmaS (sigma_s);
    bilateral_filter.setSigmaR (sigma_r);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    bilateral_filter.filter(*cloud);

    std::cout << "Done bilateral filter." << std::endl;
    // Normal Estimation
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    normal_estimation.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    normal_estimation.setRadiusSearch (0.05);

    // Compute the features
    normal_estimation.compute (*cloud_normals);

    std::cout << "Done normal estimation" << std::endl;


    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> mps;
    // Set up Organized Multi Plane Segmentation
    double mps_start = pcl::getTime ();

    mps.setMinInliers (100);
    mps.setAngularThreshold (pcl::deg2rad (2.0)); //3 degrees
    mps.setDistanceThreshold (0.02); //2cm

    mps.setInputNormals (cloud_normals);
    mps.setInputCloud (cloud);

    std::vector<pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA> > > regions;
    std::vector<pcl::ModelCoefficients> model_coefficients;
    std::vector<pcl::PointIndices> inlier_indices;  
    pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    std::vector<pcl::PointIndices> boundary_indices;

    bool use_planar_refinement_ = true;
    if (use_planar_refinement_) {
      mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
    } else {
      mps.segment (regions);//, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
    }

    double mps_end = pcl::getTime ();
    std::cout << "MPS+Refine took: " << double(mps_end - mps_start) << std::endl;

    // Pick random color for each plane
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1, 255);

    for (int i = 0; i < label_indices.size(); i++) {
      if (label_indices[i].indices.size() < 100)
        continue;
      std::cout << "Plane " << i << " has " << label_indices[i].indices.size() << " points" << std::endl;
      if (label_indices[i].indices.size() < 50)
        continue;
      uint8_t r = dis(gen), g = dis(gen), b = dis(gen);
      for (int j = 0; j < label_indices[i].indices.size(); j++) {
        int idx = label_indices[i].indices[j];
        cloud->points[idx].r = r;
        cloud->points[idx].g = g;
        cloud->points[idx].b = b;
        //cloud->points[idx].r = label_color[large_plane][0];
        //cloud->points[idx].g = label_color[large_plane][1];
        //cloud->points[idx].b = label_color[large_plane][2];
      }
    }

//-------------------- Coarse Segmentation
    pcl::PointCloud<pcl::Label>::Ptr labels_origin (new pcl::PointCloud<pcl::Label>);
    labels_origin->points.resize(cloud->points.size());
    std::cout << "Created " << labels_origin->points.size() << " labels" << std::endl;
    for (int i = 0; i < labels_origin->points.size(); i++)
      labels_origin->points[i].label = i;
    
    std::vector<bool> plane_labels;
    plane_labels.resize (cloud->points.size (), false);

    pcl::EuclideanClusterComparator<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::Ptr comparator (new pcl::EuclideanClusterComparator<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> ());
    comparator->setInputCloud (cloud);
    comparator->setInputNormals (cloud_normals);
    comparator->setLabels (labels_origin);
    comparator->setExcludeLabels (plane_labels);
    comparator->setDistanceThreshold (0.01f, false);
    std::cout << "Done initialize comparator" << std::endl;

    pcl::EuclideanPlaneCoefficientComparator<pcl::PointXYZRGBA, pcl::Normal>::Ptr plane_comparator (new pcl::EuclideanPlaneCoefficientComparator<pcl::PointXYZRGBA, pcl::Normal> ());
    plane_comparator->setInputCloud (cloud);
    plane_comparator->setInputNormals (cloud_normals);
    plane_comparator->setDistanceThreshold (0.02f, false);
    plane_comparator->setAngularThreshold( pcl::deg2rad (2.0f) );
 
    // Run segmentation
//    pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZRGBA, pcl::Label> segmentation(comparator);
    pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZRGBA, pcl::Label> segmentation(plane_comparator);
  
    pcl::PointCloud<pcl::Label> labels_final;
    std::vector<pcl::PointIndices> region_indices;
    segmentation.setInputCloud (cloud);
    segmentation.segment (labels_final, region_indices);

    std::cout << "Detected " << region_indices.size() << " planes" << std::endl;
/*
    for (int i = 0; i < region_indices.size(); i++) {
      if (region_indices[i].indices.size() < 10)
        continue;
      std::cout << "Plane " << i << " has " << region_indices[i].indices.size() << " points" << std::endl;
      if (region_indices[i].indices.size() < 50)
        continue;
      uint8_t r = dis(gen), g = dis(gen), b = dis(gen);
      for (int j = 0; j < region_indices[i].indices.size(); j++) {
        int idx = region_indices[i].indices[j];
        cloud->points[idx].r = r; 
        cloud->points[idx].g = g;
        cloud->points[idx].b = b;
        //cloud->points[idx].r = label_color[large_plane][0];
        //cloud->points[idx].g = label_color[large_plane][1];
        //cloud->points[idx].b = label_color[large_plane][2];
      }
    }
*/    
    // Create Viewport to display segmentation results
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Viewer"));
    int view1(0);
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, view1);
    viewer->addText("Radius: 0.01", 10, 10, "v1 text", view1); 
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "segmented cloud", view1);

    // Create Viewport to display Normal results
    // Get color of normal
    for (int i = 0; i < cloud_normals->points.size (); i++) {
      // Paint cloud using normal maps
      cloud->points[i].r = (uint8_t)((cloud_normals->points[i].normal_y + 1) / 2 * 255);
      cloud->points[i].g = (uint8_t)((cloud_normals->points[i].normal_x + 1) / 2 * 255);
      cloud->points[i].b = (uint8_t)((cloud_normals->points[i].normal_z + 1) / 2 * 255);
    }
    int view2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, view2);
    viewer->addText("Radius: 0.01", 10, 10, "v2 text", view2); 
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> normal_rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, normal_rgb, "cloud normal", view2);

    //blocks until the cloud is actually rendered
/*    viewer.showCloud(cloud);
    
    while (!viewer.wasStopped ()) {
      user_data++;
    }
*/
    // -----Main loop-----
    while (!viewer->wasStopped ()) {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}
