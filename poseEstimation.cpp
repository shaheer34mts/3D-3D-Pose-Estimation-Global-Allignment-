#include <iostream>
#include "poseEstimation.hpp"

pcl::PointCloud<PointT>::Ptr PoseEstimation::read_clouds(string String){
    pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr object(new pcl::PointCloud<PointT>);
    pcl::PCDReader reader;
    if (String=="scene"){
        reader.read ("scene.pcd", *scene);
        return scene;}
    else if (String== "object"){
        reader.read ("object.pcd", *object);
        return object;
    }
    else {
        cout<<"Wrong Argument !!"<<endl;
        return 0;
    }
}

void PoseEstimation::view_clouds(pcl::PointCloud<PointT>::Ptr object, pcl::PointCloud<PointT>::Ptr scene){
     pcl::visualization::PCLVisualizer MyVisualizer ("Clouds Before Allignment");
     MyVisualizer.addPointCloud<PointT> (object, pcl::visualization::PointCloudColorHandlerCustom<PointT>(object, 255, 0, 0),"object");
     MyVisualizer.addPointCloud<PointT> (scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0, 0, 255),"scene");
     MyVisualizer.spin();
}

void PoseEstimation::filter_clouds (pcl::PointCloud<PointT>::Ptr cloud_scene){
     pcl::PassThrough<PointT> MyFilter1;
     MyFilter1.setInputCloud (cloud_scene);
     MyFilter1.setFilterFieldName ("z");
     MyFilter1.setFilterLimitsNegative (true);
     MyFilter1.setFilterLimits (-10, -1.7);
     MyFilter1.filter (*cloud_scene);
     pcl::PCDWriter writer;
     pcl::PassThrough<PointT> pass2;
     pass2.setInputCloud (cloud_scene);
     pass2.setFilterFieldName ("y");
     pass2.setFilterLimitsNegative (true);
     pass2.setFilterLimits (-10, 0.16);
     pass2.filter (*cloud_scene);
     writer.write ("yzfiltercloud.pcd", *cloud_scene);
}

void PoseEstimation::Estimate_SurfaceNormals (pcl::PointCloud<PointT>::Ptr object, pcl::PointCloud<PointT>::Ptr scene){
     pcl::NormalEstimation<PointT,PointT> MyEstimator;
     MyEstimator.setKSearch(10);
     MyEstimator.setInputCloud(object);
     MyEstimator.compute(*object);
     MyEstimator.setInputCloud(scene);
     MyEstimator.compute(*scene);
}
void PoseEstimation::Compute_FPFH_Features (pcl::PointCloud<PointT>::Ptr object, pcl::PointCloud<PointT>::Ptr scene, pcl::PointCloud<FeatureT>::Ptr object_features, pcl::PointCloud<FeatureT>::Ptr scene_features){
     pcl::ScopeTime t ("FPFH Features");{
     pcl::FPFHEstimation<PointT, PointT, FeatureT> fpfh;
     fpfh.setKSearch(10);
     fpfh.setInputCloud(object);
     fpfh.setInputNormals(object);
     fpfh.compute(*object_features);
     fpfh.setInputCloud(scene);
     fpfh.setInputNormals(scene);
     fpfh.compute(*scene_features);
    }

}
void PoseEstimation::Find_Matches(pcl::PointCloud<PointT>::Ptr object, pcl::PointCloud<PointT>::Ptr scene, pcl::PointCloud<FeatureT>::Ptr object_features, pcl::PointCloud<FeatureT>::Ptr scene_features){
    pcl::Correspondences corr(object_features->size());

    {
        pcl::ScopeTime t("Feature matches");
        for(size_t i = 0; i < object_features->size(); ++i) {
            corr[i].index_query = i;
            nearest_feature(object_features->points[i], *scene_features, corr[i].index_match, corr[i].distance);
        }
    }

    pcl::visualization::PCLVisualizer v("Feature Matches");
    v.addPointCloud<PointT>(object, pcl::visualization::PointCloudColorHandlerCustom<PointT>(object, 0, 255, 0), "object");
    v.addPointCloud<PointT>(scene, pcl::visualization::PointCloudColorHandlerCustom<PointT>(scene, 0, 0, 255),"scene");
    v.addCorrespondences<PointT>(object, scene, corr, 1);
    v.spin();
}





void PoseEstimation::nearest_feature(const FeatureT& query, const pcl::PointCloud<FeatureT>& target, int &idx, float &distsq) {
    idx = 0;
    distsq = dist_sq(query, target[0]);
    for(size_t i = 1; i < target.size(); ++i) {
        const float disti = dist_sq(query, target[i]);
        if(disti <= distsq+4.5) {
            idx = i;
            distsq = disti;
        }
    }
}
float PoseEstimation::dist_sq(const FeatureT &query, const FeatureT &target){
    float result = 0.0;
    for(int i = 0; i < FeatureT::descriptorSize(); ++i) {
        const float diff = reinterpret_cast<const float*>(&query)[i] - reinterpret_cast<const float*>(&target)[i];
        result += (diff * diff);
    }

    return result;
}

