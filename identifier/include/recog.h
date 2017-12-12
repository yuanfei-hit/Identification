#pragma once
#include "head.h"
#include "model.h"

class Recog
{
public:
    //-----     default constructor     -----
    Recog();
    //-----     constructor, used to read models and database     -----
    Recog(vector<string> models_filename, string database_filename);
    //-----     constructor, used to read database     -----
    Recog(string database_filename);
    //-----     not implemented     -----
    ~Recog();
    //-----     do recognition given visual, tactile and stiffness, stiffness has not been used yet     -----
    bool isRecognitionSuccess(int model_id, PointCloudT& visual, PointCloudNT& tactile_0, PointCloudNT& tactile_1, PointCloudNT& tactile_2, std::vector<float>& stiffness);
    //-----     visualization     -----
    void visualizePointCloud(vector<PointCloudT::Ptr> cloud);
    void visualizePointCloud(vector<PointCloudNT::Ptr> cloud);
private:
    //-----     models     -----
    vector<Model> models;
    //-----     database     -----
    Database database;
    //-----     get searched model index in model vector     -----
    int getModelIndex(int model_id);
    //-----     ICP using Normal     ----
    std::pair<float, Eigen::Matrix4f> getAlignScoreAndTF(PointCloudT::Ptr source, PointCloudT::Ptr target, float degree);
    std::pair<float, Eigen::Matrix4f> getAlignScoreAndTF(PointCloudT::Ptr source, PointCloudT::Ptr target);
    //-----     extract slice point cloud     -----
    PointCloudT::Ptr extractSlicePointCloud(PointCloudT& cloud, float z_height);
    //-----     get stiffness similarity scores    -----
    std::vector<std::pair<int, float> > getStiffnessSimilarityScores(int model_id, std::vector<float>& stiffness);
    //-----     get slice scores     -----
    std::vector<std::pair<int, std::vector<std::pair<int, float> > > > getSliceScores(int model_id, PointCloudT& visual, PointCloudNT& tactile_0, PointCloudNT& tactile_1, PointCloudNT& tactile_2);
    //-----     get point index of z value closest to the z_height     -----
    int getClosestPointIndex(PointCloudT& cloud, float z_height);
};
