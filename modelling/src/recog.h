#pragma once
#include "head.h"
#include "model.h"

class Recog
{
public:
    //-----     deault constructor     -----
    Recog();
    //-----     constructor, used to read models and database     -----
    Recog(vector<string> models_filename, string database_filename);
    //-----     search a model ID in database     -----
    void search(int id);
    //-----     not implemented     -----
    ~Recog();
    //-----     info     -----
    void info();
    //-----     do recognition given visual, tactile and stiffness, stiffness has not been used yet     -----
    vector<float> doRecognition(PointCloudT visual, PointCloudNT tactile, double stiffness);
private:
    //-----     model ID     -----
    int id;
    //-----     searched model's corresponding models' IDs     -----
    vector<vector<int> > compare_ids;
    //-----     searched model's corresponding models' spans     -----
    vector<pair<float, float> > compare_spans;
    //-----     searched model's corresponding models' heights     -----
    vector<float> compare_heights;
    //-----     searched model's corresponding models' slices' indexes     -----
    vector<vector<int> > compare_slice_index;
    //-----     readed models     -----
    vector<Model> models;
    //-----     database object     -----
    Database database;
    //-----     get searched model index in database     -----
    int getModelIndex(int id);
    //-----     get span index via height     -----
    int getSpanIndex(float height);
    //-----     ICP without Normal     -----
    double getICPScore(PointCloudT::Ptr source, PointCloudT::Ptr target);
    void rotatePointCloud(PointCloudT::Ptr &slice, float degree, char mode);
    //-----     ICP using Normal     ----
    float getICPScore(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2, float degree);
};
