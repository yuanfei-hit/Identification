#pragma once
#include "head.h"
#include "model.h"

class ModelBuilder
{
public:
    //-----     default constructor     -----
    ModelBuilder(){};
    //-----     constructor     -----
    ModelBuilder(vector<string> filenames);
    //-----     not implemented     -----
    ~ModelBuilder(){};
private:
    //-----     model pair data structure     -----
    map<pair<int, int>, pair<bool, vector<pair<int, float> > > > model_pair_database;
    //-----     name of model pair database     -----
    string model_pair_database_filename;
    //-----     models structure     -----
    vector<Model> models;
    //-----     get one-to-many slice scores for similar models     -----
	map<int, pair<vector<int>, vector<vector<pair<int, float> > > > > getSliceScoresForSimilarModel();
	//-----     output model pair info     -----
	void outputModelPairInfo();
	//-----     get similarity among models     -----
	bool isSimilar(Model model1, Model model2);
	//-----     save model pair database     -----
	void saveModelPairDatabase(string filename);
	//-----     read model pair database     -----
	void readModelPairDatabase(string filename);
    //-----     build model pair database     -----
    void buildModelPairDatabase();
    //-----     build similar database     -----
    void buildSimilarModelDatabase();
    //-----     get optimal slice ID      -----
    vector<int> getOptimalSliceID(vector<vector<int> > slice_index, vector<vector<float> > slice_score, float slice_score_threshold, int run_times);
    //-----     ICP using Normal     -----
    float getAlignScore(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2, float degree);

    void rotatePointCloud(PointCloudT::Ptr &slice, float degree, char mode);
    void visualizePointCloud ( vector<PointCloudT::Ptr> cloud, vector<RGB> rgb);
    void visualizePointCloud ( PointCloudT::Ptr cloud, RGB rgb );
    void visualizePointCloud ( vector<PointCloudNT::Ptr> cloud);
};

// node class for slice optimisation based on fingers (no longer using)
class Node
{
public:
    vector<int> ids;
    float height;
    Slice slice;
    Node* father;
    Node* left;
    Node* right;
    Node(Slice data){
        this->ids.push_back(data.id);
        this->height = data.height; 
        this->slice = data;
        this->father = NULL; 
        this->left = NULL; 
        this->right = NULL;
        };
};

// tree class for previous nodes (no longer using)
class Tree
{
public:
    Node* head;
    Tree();
    void ins(Slice data);
    Node* search(float height, bool& isExist);
    Node* search(pair<float, float> span, bool& isExist);
    void print(Node* node);
    vector<float> getValues();
    void getValues(vector<float> &height, vector<pair<float, float> > &span, vector<vector<int> > &ids);
};
