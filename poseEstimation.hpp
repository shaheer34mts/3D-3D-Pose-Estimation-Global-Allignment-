#ifndef POSEESTIMATION_HPP
#define POSEESTIMATION_HPP

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/common/random.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/features/pfh.h>
#include <pcl/features/vfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>

using namespace std;
typedef pcl::PointNormal PointT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::VFHSignature308 FeatureT1;
typedef pcl::PFHSignature125 FeatureT2;



class PoseEstimation {
public:
    pcl::Correspondences _corr1;
    pcl::PointCloud<PointT>::Ptr read_clouds(string);
    void view_clouds(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<PointT>::Ptr);
    void filter_clouds(pcl::PointCloud<PointT>::Ptr);
    void Estimate_SurfaceNormals(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<PointT>::Ptr);
    void Compute_FPFH_Features(pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<FeatureT>::Ptr, pcl::PointCloud<FeatureT>::Ptr);
    void Find_Matches (pcl::PointCloud<PointT>::Ptr, pcl::PointCloud<PointT>::Ptr,pcl::PointCloud<FeatureT>::Ptr, pcl::PointCloud<FeatureT>::Ptr);
    void Apply_Ransac();
private:

    void nearest_feature(const FeatureT& query, const pcl::PointCloud<FeatureT>& target, int &idx, float &distsq);
    float dist_sq(const FeatureT& query, const FeatureT& target);





};
#endif
