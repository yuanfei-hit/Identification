#pragma once
#include "head.h"
#include "properties.h"
#include "model.h"

// class ExtractedFeature
// {
// public:
// 	vector<PointCloudT::Ptr> extracted_slices;
// 	vector<pair<float, float> > slices_span;
// 	vector<PointCloudT::Ptr> key_points;
// 	ExtractedFeature(vector<PointCloudT::Ptr> extracted_slices, vector<pair<float, float> > slices_span, vector<PointCloudT::Ptr> key_points);
// };
class FeatureExtraction
{
public:
	//-----     model height      -----
	float height;
	//-----     extracted slices     -----
	vector<PointCloudT::Ptr> extracted_slices;
	//-----     slices' corresponding spans      -----
	vector<pair<float, float> > slices_span;
	//-----     unused slice index      -----
	vector<int> trimmed_slice_index;
	//-----     slices' corresponding key points      -----
	vector<PointCloudT::Ptr> key_points;
	//-----     default constructor     -----
	FeatureExtraction();
	//-----     constructor     -----
    FeatureExtraction(int id, string filename, string config);
	//-----     get key points (can be private)     -----
 	PointCloudT::Ptr getKeyPoint(PointCloudT::Ptr cloud, int bins);  
	//-----     not implemented     ----- 
	~FeatureExtraction();

private:
	//-----     model PCD file name     ----- 
    string filename;
	//-----     config filename     ----- 
    string config;
	//-----     model PCD     ----- 
    PointCloudT::Ptr cloud;
	//-----     slice size     -----
	int slices_size;
	//-----     interval between slices (properties from yaml)     -----
	float interval;
	//-----     searching span between slices (properties from yaml)     -----
	float epsilon;
	//-----     basic searching height between slices, changed later (properties from yaml)     -----
	float search_base;
	//-----     basic searching height slices (properties from yaml)     -----
	float base;
	//-----     max number of slices (properties from yaml)     -----
	int search_num;
	//-----     model ID     ----- 
	int id;
	//-----     ICP threshold between slices (properties from yaml)     -----
	float threshold;
	//-----     max slice points (properties from yaml)     -----
	int slice_points;
	//-----     valid searching height (properties from yaml)     ----- 
	float valid_height;
	//-----     ICP order (properties from yaml)     ----- 
	int order;
	//-----     model name     ----- 
	string name;
	//-----     all slices     -----
    vector<PointCloudT::Ptr> slices;
	//-----     read config file     ----- 
    void readConfig();
	//-----     load point cloud     ----- 
    void loadPointCloud();
	//-----     extract all slices     -----
    void extractSlices();
	//-----     save point cloud     ----- 
	void savePointCloud(vector<PointCloudT::Ptr> clouds, string prefix);
	//-----     select slices from all slices     -----
	void selectSlices();
	//-----     visualization     ----- 
	void visualizeSlices(vector<PointCloudT::Ptr> visual);
	void visualizeSlices(vector<PointCloudNT::Ptr> visual);
	//-----     ICP without Normal     ----- 
	float getICPScore(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2, float degree);
	//-----     ICP with Normal     -----
	float getAlignScore(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2, float degree);
	//-----     calculate KL divergence, no longer using     ----- 
	float KL(vector<float> p, vector<float> q);
	//-----     calculate KL divergence     ----- 
	float getKLDivergence(vector<float> data, vector<float> predict, int bins);
	//-----     separate coordinates     ----- 
	map<string, vector<float> > seperateXY(vector<vector<float> > data);
	//-----     separate data into two parts     ----- 
	map<string, vector<float> > seperateData(vector<vector<float> > data);
	
	float min(vector<float> data);
	float max(vector<float> data);
	float min(float data1, float data2);
	float max(float data1, float data2);

	//-----     normalize, not used here, if there are few samples, this method may be needed     ----- 
	void minusMean(vector<float> &data);
	//-----     get entropy, no longer using     ----- 
	float getEntropy(vector<float> data1, vector<float> data2, int bins);

	void writeFile(string filename, vector<float> data);
	void writeFile(string filename, vector<int> data);
	void writeFile(string filename, vector<pair<float, float> > data);
	void rotatePointCloud(PointCloudT::Ptr &cloud, float degree, char mode);	
	void visualizePointCloud ( PointCloudT::Ptr cloud, RGB rgb );
	void visualizePointCloud ( PointCloudT::Ptr* clouds, RGB* rgbs, int number);

	//-----     extract key points function     ----- 
	void extractKeyPoints();
	//-----     get key points by rotate point cloud 180 degrees twice     ----- 
	PointCloudT::Ptr getAllKeyPoint(PointCloudT::Ptr cloud, int bins);

	// PointCloudT::Ptr getKeyPoint(PointCloudT::Ptr cloud, int bins);
	// void convertPointCloudToPolar(PointCloudT::Ptr &cloud, PointCloudT::Ptr &polar);
	// void convertPolarToPointCloud(PointCloudT::Ptr &polar, PointCloudT::Ptr &cloud);
};
