#pragma once
#include "head.h"

// model class
class Model
{
public:
    //-----     model name     -----
    string name;
    //-----     model ID     -----
    int id;
    //-----     model height     -----
    float height;
    //-----     slice number     -----
    int slice_num;
    //-----     surface number     -----
    int raw_num;
    //-----     original surface point cloud     -----
    vector<PointCloudT::Ptr> raw;
    //-----     extracted slices     -----
    vector<PointCloudT::Ptr> slices;
    //-----     slices' corresponding spans     -----
    vector<pair<float, float> > spans;
    //-----     unused slice index      -----
    vector<int> trimmed_slice_index;
    //-----     slices' corresponding key points     -----
	vector<PointCloudT::Ptr> keys;
    //-----     texture: stiffness, has not used yet     -----
    float texture;
    //-----     constructor, used in saving database     -----
    Model(int id, string name, vector<PointCloudT::Ptr> raw, vector<PointCloudT::Ptr> slices, vector<PointCloudT::Ptr> keys, vector<pair<float, float> > spans, vector<int> trimmed_slice_index, float height);
    //-----     constructor, used in reading databse     ----
    Model(string filename);
    //-----     default constructor     -----
    Model();
    //-----     not implemented     -----
    ~Model();
    //-----     read models     -----
    void readModel(string filename);
    //-----     save models     -----
    void saveModel(string filename);

    void visualizeSlices(vector<PointCloudT::Ptr> visual);
    //-----     info     -----
    void info();
};

// database class
class Database
{
public:
    //-----     model's ID     -----
    vector<int> model_ids;
    //-----     similar surface models' ID     -----
    vector<vector<int> > similar_surface_model_ids;
    //-----     similar slice models' ID     -----
    vector<vector<int> > similar_slice_model_ids;
    //-----     similar stiffness models' ID     -----
    vector<vector<int> > similar_stiffness_model_ids;
    //-----     optimal slice ID     -----
    vector<vector<int> > slice_ids;
    //-----     optimal slice heights     -----
    vector<vector<float> > slice_heights;
    //-----     distinguished similar models' ID     -----
    vector<vector<vector<int> > > dist_similar_surface_model_ids;
    //-----     default constructor     -----
    Database();
    //-----     constructor, used when read database     -----
    Database(string filename);
    //-----     not implemented     -----
    ~Database();
    //-----     add comparison information of a specific model     -----
    void insert(int model_id,
    		    vector<int> similar_surface_model_ids,
				vector<int> similar_slice_model_ids,
				vector<int> similar_stiffness_model_ids,
				vector<int> slice_ids,
				vector<float> slice_heights,
				vector<vector<int> > dist_similar_surface_model_ids);
    //-----     save database     -----
    void saveDatabase(string filename);
    //-----     read database     -----
    void readDatabase(string filename);
    //-----     get index of model, for security purpose     -----
    int search(int id);
    //-----     output size of database     -----
    int size();
    //-----     info     -----
    void info();

private:
    //-----     database size     -----
    int database_size;
};
