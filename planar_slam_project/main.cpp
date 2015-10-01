#include <iostream>
#include <random>

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
 
int 
main (int argc, char** arg)
{
    // Load data
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_1_origin (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_2_origin (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile("../data/my_room/1.pcd", *cloud_1_origin);
    pcl::io::loadPCDFile("../data/my_room/2.pcd", *cloud_2_origin);

    for (size_t i = 0; i < cloud_2_origin->points.size (); ++i) {
      cloud_2_origin->points[i].x = cloud_2_origin->points[i].x + 0.6f;
      cloud_2_origin->points[i].y = cloud_2_origin->points[i].y + 0.9f;
    }
    
   std::cout << "Transformed " << cloud_2_origin->points.size() << " points" << std::endl;
    
// Do ICP
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr c1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr c2 (new pcl::PointCloud<pcl::PointXYZRGBA>);
    c1->width    = cloud_1_origin->width;
    c1->height   = cloud_1_origin->height;
    c1->is_dense = cloud_1_origin->is_dense;
    c1->points.resize (c1->width * c1->height);

    for (size_t i = 0; i < c1->points.size (); ++i) {
      c1->points[i].x = cloud_1_origin->points[i].x;
      c1->points[i].y = cloud_1_origin->points[i].y;
      c1->points[i].z = cloud_1_origin->points[i].z;
    }

    std::cout << "Doing voxelgrid ..." << std::endl;
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;
    voxel_grid.setInputCloud (c1);
    voxel_grid.setLeafSize (0.05, 0.05, 0.05);
    voxel_grid.filter(*c1);

    *c2 = *c1;
    for (size_t i = 0; i < c2->points.size (); ++i) {
      c2->points[i].x += 1.0f;
      c2->points[i].y += 0.5f;
      c2->points[i].z += 0.7f;
    }
    std::cout << "Copied cloud to c1, c2\n";

// Do Normal ICP
/*
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    icp.setEuclideanFitnessEpsilon (1);

    icp.setInputSource(c1);
    icp.setInputTarget(c2);

    pcl::PointCloud<pcl::PointXYZRGBA> Final;
    icp.align(Final);
*/

// Do ICP Non Linear

    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src (new pcl::PointCloud<pcl::PointNormal> ());
    pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt (new pcl::PointCloud<pcl::PointNormal> ());

//    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
//    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::PointNormal> norm_est;
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
    norm_est.setSearchMethod (kdtree);
    norm_est.setKSearch (30);

    norm_est.setInputCloud (c1);
    norm_est.compute (*points_with_normals_src);
    pcl::copyPointCloud (*c1, *points_with_normals_src);

    norm_est.setInputCloud (c2);
    norm_est.compute (*points_with_normals_tgt);
    pcl::copyPointCloud (*c2, *points_with_normals_tgt);
 
    pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> icp;
    //icp.setTransformationEpsilon (1e-6);

    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    //icp.setMaxCorrespondenceDistance (0.1);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (100);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (1e-7);
    // Set the euclidean distance difference epsilon (criterion 3)
    //icp.setEuclideanFitnessEpsilon (1);


    //icp.setMaxCorrespondenceDistance (0.1);
    //reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

    icp.setInputSource (points_with_normals_src);
    icp.setInputTarget (points_with_normals_tgt);

    pcl::PointCloud<pcl::PointNormal> Final;
    icp.align(Final);

//  Visualize ICP
    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << "Final cloud has " << Final.points.size() << " points.\n";
    std::cout << icp.getFinalTransformation() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::transformPointCloud (*c1, *output, icp.getFinalTransformation()); 


#ifdef VIZ
    pcl::visualization::PCLVisualizer *p;
    p = new pcl::visualization::PCLVisualizer ("Registration");
    int vp_1, vp_2;
    p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> tgt_h (c2, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> src_h (c1, 255, 0, 0);
    p->addPointCloud (c2, tgt_h, "vp1_target", vp_1);
    p->addPointCloud (c1, src_h, "vp1_source", vp_1);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> cloud_tgt_h (c2, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> cloud_src_h (output, 255, 0, 0);
    p->addPointCloud (c2, cloud_tgt_h, "target", vp_2);
    p->addPointCloud (output, cloud_src_h, "source", vp_2);
    p->spin();
#endif


// -------------------------------------------
//  add point to a single one
   std::cout << "points before " << c2->points.size() << std::endl;
   *c2 += *output; 
   std::cout << "points after " << c2->points.size() << std::endl;

   voxel_grid.setInputCloud (c2);
   voxel_grid.setLeafSize (0.05, 0.05, 0.05);
   voxel_grid.filter(*c2);
   std::cout << "points after " << c2->points.size() << std::endl;

// ------------------------------------------

    //pcl::visualization::CloudViewer viewer("Cloud Viewer");
    
    float sigma_s = 10.f, sigma_r = 0.5f;
    pcl::FastBilateralFilter<pcl::PointXYZRGBA> bilateral_filter;
    bilateral_filter.setInputCloud (cloud_1_origin);
//    bilateral_filter.setHalfSize (sigma_s);
//    bilateral_filter.setStdDev (sigma_r);
    bilateral_filter.setSigmaS (sigma_s);
    bilateral_filter.setSigmaR (sigma_r);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    bilateral_filter.filter(*cloud);

    std::cout << "Done bilateral filter." << std::endl;
/*
    std::cout << "Doing voxelgrid ..." << std::endl;
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxel_grid;
    voxel_grid.setInputCloud (cloud);
    voxel_grid.setLeafSize (0.5, 0.5, 0.5);
    voxel_grid.filter(*cloud);
*/

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
    normal_estimation.setRadiusSearch (0.08);

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

    std::cout << "Found " << label_indices.size() << " planes " << std::endl;
    std::cout << "Found " << model_coefficients.size() << " models" << std::endl;
    std::cout << "Found " << regions.size() << " regions" << std::endl;

// Print plane information
    for (size_t i = 0; i < regions.size (); i++) {
      Eigen::Vector3f centroid = regions[i].getCentroid ();
      Eigen::Vector4f model = regions[i].getCoefficients ();
      pcl::PointCloud<pcl::PointXYZRGBA> boundary_cloud;
      boundary_cloud.points = regions[i].getContour ();
      printf ("Centroid: (%f, %f, %f)\n  Coefficients: (%f, %f, %f, %f)\n Inliers: %d %d\n",
          centroid[0], centroid[1], centroid[2],
          model[0], model[1], model[2], model[3],
          boundary_cloud.points.size (), inlier_indices[i].indices.size());
      uint8_t r = dis(gen), g = dis(gen), b = dis(gen);
      for (int j = 0; j < inlier_indices[i].indices.size(); ++j) {
        int idx = inlier_indices[i].indices[j];
        cloud->points[idx].r = r;
        cloud->points[idx].g = g;
        cloud->points[idx].b = b;        
      }
    }

/*    
// All segmentations
    for (int i = 0; i < label_indices.size(); i++) {
      if (label_indices[i].indices.size() < 200)
        continue;
      std::cout << "Plane " << i << " has " << label_indices[i].indices.size() << " points" << std::endl;

      std::cout << "Plane Coefficient: \n";
      Eigen::Vector4f model = regions[i].getCoefficients();
      for (int j = 0; j < model.size(); j++)
        std::cout << model[j] << "  ";
      std::cout << std::endl;

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
*/
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
#ifdef VIZ
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
#endif

    return 0;
}
