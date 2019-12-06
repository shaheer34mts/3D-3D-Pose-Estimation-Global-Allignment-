#include "poseEstimation.hpp"
#include <iostream>

using namespace std;
int main(int argc, char** argv)
{
    PoseEstimation MyCloud;
    pcl::PointCloud<PointT>::Ptr object = MyCloud.read_clouds("object");
    pcl::PointCloud<PointT>::Ptr scene = MyCloud.read_clouds("scene");
    pcl::PointCloud<FeatureT>::Ptr object_features(new pcl::PointCloud<FeatureT>);
    pcl::PointCloud<FeatureT>::Ptr scene_features(new pcl::PointCloud<FeatureT>);
    MyCloud.view_clouds(object,scene);
    MyCloud.filter_clouds(scene);
    MyCloud.view_clouds(object,scene);
    MyCloud.Estimate_SurfaceNormals(object, scene);
    MyCloud.Compute_FPFH_Features(object, scene, object_features, scene_features);
    MyCloud.Find_Matches(object, scene, object_features, scene_features);

return 0;
}
