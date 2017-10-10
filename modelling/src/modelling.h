#pragma once
#include "head.h"
#include "properties.h"
#include "extract_feature.h"

// modelling class
class Modelling
{
public:
    //-----     info     -----
    Modelling(int id, string config, string name);
    //-----     info     -----
    ~Modelling();    
    //-----     info     -----
    PointCloudT::Ptr getModel();
    //-----     info     -----
    int getCloudsNum(){return this->clouds_num;};
    //-----     info     -----
    vector<PointCloudT::Ptr> getClouds(){return this->clouds;};

private:
    //-----     initialize function     -----
    void initialize();
    //-----     mdoel name     -----
    string name;
    //-----     surface clouds     -----
    vector<PointCloudT::Ptr> clouds;
    //-----     surface clouds filenames     -----
    vector<string> clouds_filenames;
    //-----     rotation degree (properties from yaml)     -----
    float rotate_degree;
    //-----     pose degree (properties from yaml)     -----
    float pose_degree;
    //-----     x translation (properties from yaml)     -----
    float x_bias;
    //-----     pitch (properties from yaml)     -----
    float x_degree;
    //-----     y translation (properties from yaml)     -----
    float y_bias;
    //-----     roll (properties from yaml)     -----
    float y_degree;
    //-----     z translation (properties from yaml)     -----
    float z_bias;
    //-----     yaw (properties from yaml)     -----
    float z_degree;
    //-----     transformation matrix filename (properties from yaml)     -----
    string matrix_filename;
    //-----     config filename     -----
    string properties_filename;
    //-----     transformation matrix     -----
    Eigen::Matrix4f matrix;
    //-----     clouds number     -----
    int clouds_num;
    //-----     model ID     -----
    int id;
    //-----     properties objects     -----
    SamplingProperties sampling_properties;
    ModellingProperties modelling_properties;
    PoissonProperties poisson_properties;
    // VoxelGridProperties voxel_properties;
    //-----     load point cloud     -----
    PointCloudT::Ptr loadPointCloud ( string filename );
    //-----     read matrix     -----
    Eigen::Matrix4f readMatrix();
    //-----     using properties rotate the point cloud     -----
    void preProcessPointCloud(PointCloudT::Ptr &cloud, Eigen::Matrix4f matrix, int index);
    //-----     poisson sampling, no longer used     -----
    void smoothAndSampling(PointCloudT::Ptr &cloud, float search_radius, bool polynomial_fit, int polynomial_order, float radius_search, int depth);
    //-----     poisson sampling     -----
    void poissonSampling(PointCloudT::Ptr &cloud, int k_search, bool confidence, int degree, int depth, int ISO, bool manifold, bool polygon, float smooth, float scale, int solver, int min_depth);
    //-----     outlier removing filter     -----
    void removeOutlier(PointCloudT::Ptr &cloud, int meanK, int threshold);
    //-----     voxel grid filter     -----
    void voxelGridFilter(PointCloudT::Ptr &cloud, float size);
    
    void rotatePointCloud(PointCloudT::Ptr &cloud, float degree, char mode);
    // void visualizePointCloud(PointCloudT::Ptr cloud, RGB rgb);
    // void visualizePointCloud(PointCloudT::Ptr* cloud, RGB* rgb, int number);
    void visualizePointCloud ( vector<PointCloudT::Ptr> cloud);
};