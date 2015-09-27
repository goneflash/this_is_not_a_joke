#ifndef PLANAR_SLAM_PREPROCESSOR_H
#define PLANAR_SLAM_PREPROCESSOR_H

namespace planar_slam {

template <
class Preprocessor {
public:
  Preprocessor() : use_bilaterl_filter_(false), 
                   use_voxel_grid_(false),
                   compute_normal_(false) { };
  ~Preprocessor();

  void initBilateralFilter(float sigma_s = 10.f, float sigma_r = 0.5f) {
    use_bilater_filter_ = true; 
    bf_sigma_s_ = sigma_s;
    bf_sigma_r_ = sigma_r;
  }
  void initVoxelGrid() { use_voxel_grid_ = true; }
  void initNormalEstimation(float radius_search = 0.05) {
    compute_normal_ = true;
    ne_radius_search_ = radius_search;
    normal_estimation.setRadiusSearch (ne_radius_search_);
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
    normal_estimation_.setSearchMethod (tree);
  }

  bool process();
  bool getProcessedCloud();
  bool getProcessedNormal();
  
private:
  bool applyBilateralFilter();
  bool applyVoxelGrid();
  bool estimateNormal();

  bool use_bilateral_filter_;
  float bf_sigma_s_, bf_sigma_r_;
  pcl::FastBilateralFilter<pcl::PointXYZRGBA> bilateral_filter_;

  bool use_voxel_grid_;

  bool compute_normal_;
  float ne_radius_search_;
  pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimation_;

};

} // namespace planar_slam

#endif
