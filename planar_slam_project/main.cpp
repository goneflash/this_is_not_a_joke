#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/kdtree/kdtree_flann.h>
    
int user_data;
    
int 
main (int argc, char** arg)
{
    // Load data
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_origin (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile("../data/my_room/1.pcd", *cloud_origin);
    
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    
    float sigma_s = 10.f, sigma_r = 0.1f;
    pcl::FastBilateralFilter<pcl::PointXYZRGBA> bilateral_filter;
    bilateral_filter.setInputCloud (cloud_origin);
//    bilateral_filter.setHalfSize (sigma_s);
//    bilateral_filter.setStdDev (sigma_r);
    bilateral_filter.setSigmaS (sigma_s);
    bilateral_filter.setSigmaR (sigma_r);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    bilateral_filter.filter(*cloud);



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
    normal_estimation.setRadiusSearch (0.03);

    // Compute the features
    normal_estimation.compute (*cloud_normals);


    for (int i = 0; i < cloud_normals->points.size (); i++) {
//      std::cout << "r  : " << cloud->points[i].r << " g: " << cloud->points[i].g << " b: " << cloud->points[i].b << std::endl;
//      std::cout << "nx : " << cloud_normals->points[i].normal_x << " ny : " << cloud_normals->points[i].normal_y;
//      std::cout << " nz : " << cloud_normals->points[i].normal_z << std::endl;
      cloud->points[i].r = (uint8_t)((cloud_normals->points[i].normal_y + 1) / 2 * 255);
      cloud->points[i].g = (uint8_t)((cloud_normals->points[i].normal_x + 1) / 2 * 255);
      cloud->points[i].b = (uint8_t)((cloud_normals->points[i].normal_z + 1) / 2 * 255);
    }

    //blocks until the cloud is actually rendered
    viewer.showCloud(cloud);
    
    while (!viewer.wasStopped ()) {
      user_data++;
    }
    return 0;
}
