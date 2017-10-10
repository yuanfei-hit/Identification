#include "recog.h"
void visualizeSlices(vector<PointCloudT::Ptr> visual)
{
    return;
   RGB rgbs[] = {
            RGB(255,   0,   0),
            RGB(0, 0,   255),
            RGB(255, 255,   0),
            RGB(  0, 255,   0),
            RGB(  0, 127, 255),
            RGB(  0,   0, 255),
            RGB(139,   0, 255),
            RGB(255, 255, 255),
            RGB(255, 255,   0),
			RGB(255, 127,   0),
			RGB(255, 127, 127),
			RGB(255, 127, 255)};
	pcl::visualization::PCLVisualizer viewer("cloud");
	for (int i = 0; i < visual.size(); i++)
	{
		ColorHandlerT handle (visual[i], rgbs[i%12].getR(), rgbs[i%12].getG(), rgbs[i%12].getB());
		stringstream ss;
		string index;
		ss << i;
		ss >> index;
		viewer.addPointCloud (visual[i], handle, "cloud_visualization"+index);
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_visualization"+index);
	}
	viewer.addCoordinateSystem (0.1, 0, 0, 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); 
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
}

bool sortICPPair(pair<int, double> pair1, pair<int, double> pair2)
{
    return pair1.second < pair2.second;
}
bool sortID(pair<int, double> pair1, pair<int, double> pair2)
{
    return pair1.first < pair2.first;
}


Recog::Recog()
{
    this->id = -1;
}
Recog::Recog(vector<string> models_filename, string database_filename)
{
    for (int i = 0; i < models_filename.size(); i++)
        this->models.push_back(Model(models_filename[i]));
    this->database.readDatabase(database_filename);
}
void Recog::search(int id)
{
    this->id = id;
    int index = this->database.search(this->id);
    this->compare_ids = this->database.ids[index];
    this->compare_spans = this->database.spans[index];
    this->compare_heights = this->database.heights[index];

    // cout << id << endl;
    // printf("----------------ID-----------------\n");
    // for (int i = 0; i < this->compare_ids.size(); i ++)
    // {
    //     for (int j = 0; j < this->compare_ids[i].size(); j ++)
    //     {
    //         printf("%d ", this->compare_ids[i][j]);
    //     }
    //     printf("\n");
    // }
    // printf("--------------SPANS---------------------\n");
    // for (int i = 0; i < this->compare_spans.size(); i ++)
    // {
    //     printf("%f - %f \n", this->compare_spans[i].first, this->compare_spans[i].second);
    // }    

    for (int i = 0; i < this->compare_ids.size(); i++)
    {
        vector<int> slice_index_temp;
        for (int j = 0; j < this->compare_ids[i].size(); j++)
        {
            int model_index = getModelIndex(this->compare_ids[i][j]);
            slice_index_temp.push_back(this->models[model_index].search(this->compare_heights[i]));
        }
        this->compare_slice_index.push_back(slice_index_temp);
    }
}
int Recog::getModelIndex(int id)
{
    for (int i = 0; i < this->models.size(); i++)
    {
        if (this->models[i].id == id)
            return i;
    }
    return -1;
}
int Recog::getSpanIndex(float height)
{
    // printf("height is: %f \n", height);
    // printf("span is : %f - %f \n", this->compare_spans[0].first, this->compare_spans[0].second);
    for (int i = 0; i < this->compare_spans.size(); i++)
    {
        if (height <= this->compare_spans[i].second && height >= this->compare_spans[i].first )
            return i;
    }
    return -1;
}
void Recog::info()
{
    printf("[ INPUT MODEL ]: %d\n", this->id);
    for (int i = 0; i < this->compare_slice_index.size(); i ++)
    {
        for (int j = 0; j < this->compare_slice_index[i].size(); j++)
        {
            printf("[ ID ]: %d ", this->compare_ids[i][j]);
            printf("[ height ]: %f ", this->compare_heights[i]);
            printf("[ index ]: %d \n", this->compare_slice_index[i][j]);
        }
    }
}
vector<float> Recog::doRecognition(PointCloudT visual_in, PointCloudNT tactile_in, double stiffness)
{
    vector<float> icp_scores;
    PointCloudT::Ptr tactile(new PointCloudT);
    PointCloudT::Ptr visual(new PointCloudT);
    PointCloudT::Ptr whole(new PointCloudT);
    *visual = visual_in;
    copyPointCloud(tactile_in, *tactile);

    // vector<PointCloudT::Ptr> clouds;
    // clouds.push_back(visual);
    // clouds.push_back(tactile);
    // visualizeSlices(clouds);

    *whole = *visual + *tactile;
    whole->width = whole->points.size();
    map<int, PointCloudT> captured_pointcloud;
    for (int i = 0; i < whole->points.size(); i++)
    {
        int span_index = getSpanIndex(whole->points[i].z);
        // cout << span_index << endl;
        captured_pointcloud[span_index].push_back(whole->points[i]);
    }
    map<int, PointCloudT>::iterator capture_it;
    // for (capture_it = captured_pointcloud.begin(); capture_it != captured_pointcloud.end(); ++capture_it)
    // {
    //     cout << capture_it->first << endl;
    // }
    // printf("assignment process finished\n");
    map<int, PointCloudT>::iterator it;
    vector<pair<int, double> > scores;
    float self_scores = 65535;
    int self_index = getModelIndex(this->id);
    // for (int i = 0; i < this->compare_ids.size(); i ++)
    // {
    //     for (int j = 0; j < this->compare_ids[i].size(); j ++)
    //     {
    //         cout << compare_ids[i][j] << " ";
    //     }
    //     cout << endl;
    // }
    // printf("comparing process start\n");
    for (it = captured_pointcloud.begin(); it != captured_pointcloud.end(); ++it)
    {
        PointCloudT::Ptr cloud_captured(new PointCloudT);
        *cloud_captured = it->second;
        // printf("captured size: %d\n", cloud_captured->points.size());

        for (int i = 0; i < this->compare_ids[it->first].size(); i++)
        {
            PointCloudT::Ptr database_slice(new PointCloudT);
            int model_index = getModelIndex(this->compare_ids[it->first][i]);
            // printf("model id: %d\n", this->compare_ids[it->first][i]);
            *database_slice = *models[model_index].slices[this->compare_slice_index[it->first][i]];
            // printf("database slice size: %d\n", database_slice->points.size());
            // ICP
            // vector<PointCloudT::Ptr> clouds;
            // clouds.push_back(cloud_captured);
            // clouds.push_back(database_slice);
            // visualizeSlices(clouds);
            
            scores.push_back(pair<int, double>(this->compare_ids[it->first][i], getICPScore(cloud_captured, database_slice, 10.0)));
            // printf("score: %9f \n", scores[scores.size()-1]);
            // printf("##########################\n");
        }
        // cout << scores.size() << endl;


        int self_slice_index = this->models[self_index].search(cloud_captured->points[0].z);

            // vector<PointCloudT::Ptr> clouds;
            // clouds.push_back(cloud_captured);
            // clouds.push_back(this->models[self_index].slices[self_slice_index]);
            // visualizeSlices(clouds);
        float self_score_temp = getICPScore(cloud_captured, this->models[self_index].slices[self_slice_index], 10.0);
        // printf("score: %9f \n", self_score_temp);
        // printf("##########################\n");
        if (self_score_temp < self_scores)
            self_scores = self_score_temp;
    }
    // self_scores /= captured_pointcloud.size();

    scores.push_back(pair<int, double>(this->id, self_scores));
    sort(scores.begin(), scores.end(), sortID);
    for (int i = 0; i < scores.size(); i ++)
    {
        icp_scores.push_back(scores[i].second);
    }

    sort(scores.begin(), scores.end(), sortICPPair);
    printf("[ Predict Model ]: %d \n", this->id);
    for (int i = 0; i < scores.size(); i ++)
    {
        printf("[ ID: %d ,Score: %9f ] \n", scores[i].first, scores[i].second*10000);
        // printf("The score is: %f \n", scores[0].second);
    }
    printf("[ END ]\n");
    return icp_scores;
}
double Recog::getICPScore(PointCloudT::Ptr source, PointCloudT::Ptr target)
{
    float degree = 10.0;
    int times = 360.0/degree;
	PointCloudT::Ptr temp(new PointCloudT);
	*temp = *source;
	float mini = 65535;
	// Eigen::Matrix4f transformation;
    PointCloudT::Ptr show(new PointCloudT);
	for (int i = 0; i < times; i++)
	{
		rotatePointCloud(temp, degree, 'z');
		pcl::IterativeClosestPoint<PointT, PointT> icp;
		Eigen::Matrix4f matrix;
		icp.setInputSource(temp);
		icp.setInputTarget(target);
		PointCloudT align;
		PointCloudT::Ptr final(new PointCloudT);
		icp.align(align);
		*final = align;
		float score = icp.getFitnessScore();
		if (score < mini)
		{
			mini = score;
            *show = *final;
			// transformation = icp.getFinalTransformation();
		}
	}
    // vector<PointCloudT::Ptr> clouds;
    // clouds.push_back(target);
    // clouds.push_back(show);
    // visualizeSlices(clouds);

    return mini;
}
void Recog::rotatePointCloud(PointCloudT::Ptr &slice, float degree, char mode)
{
	Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    if (mode == 'z')
    {
        matrix(0,0) = cos(degree);
        matrix(0,1) = -sin(degree);
        matrix(1,0) = sin(degree);
        matrix(1,1) = cos(degree);
    }
    else if (mode == 'y')
    {
        matrix(0,0) = cos(degree);
        matrix(0,2) = -sin(degree);
        matrix(2,0) = sin(degree);
        matrix(2,2) = cos(degree);
    }
    else if (mode == 'x')
    {
        matrix(1,1) = cos(degree);
        matrix(1,2) = -sin(degree);
        matrix(2,1) = sin(degree);
        matrix(2,2) = cos(degree);
    }
    else
    {
        cout << "insert true mode index !" << endl;
    }
	pcl::transformPointCloud(*slice, *slice, matrix);
}
Recog::~Recog()
{

}
float Recog::getICPScore(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2, float degree)
{
	int times = 360.0/degree;
	PointCloudT::Ptr temp(new PointCloudT);
	*temp = *cloud1;
	float mini = 65535;
	vector<float> saveScore;
	// Eigen::Matrix4f transformation;
    PointCloudT::Ptr show(new PointCloudT);

	for (int i = 0; i < times; i++)
	{
		rotatePointCloud(temp, degree, 'z');

		PointCloudNT::Ptr object (new PointCloudNT);
		PointCloudNT::Ptr object_aligned (new PointCloudNT);
		PointCloudNT::Ptr scene (new PointCloudNT);
		FeatureCloudT::Ptr object_features (new FeatureCloudT);
		FeatureCloudT::Ptr scene_features (new FeatureCloudT);

		copyPointCloud (*temp, *object);
		copyPointCloud (*cloud2, *scene);

		pcl::VoxelGrid<PointNT> grid;
		const float leaf = 0.005f;
		grid.setLeafSize (leaf, leaf, leaf);

		pcl::NormalEstimationOMP<PointNT,PointNT> nest;
		nest.setRadiusSearch (0.03);
		nest.setInputCloud (scene);
		nest.compute (*scene);
		nest.setInputCloud (object);
		nest.compute (*object);

		// Estimate features
	//	pcl::console::print_highlight ("Estimating features...\n");
		FeatureEstimationT fest;
		fest.setRadiusSearch (0.04);
		fest.setInputCloud (object);
		fest.setInputNormals (object);
		fest.compute (*object_features);
		fest.setInputCloud (scene);
		fest.setInputNormals (scene);
		fest.compute (*scene_features);

		// Perform alignment
	//	pcl::console::print_highlight ("Starting alignment...\n");
		pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
		align.setInputSource (object);
		align.setSourceFeatures (object_features);
		align.setInputTarget (scene);
		align.setTargetFeatures (scene_features);
		align.setMaximumIterations (50000); // Number of RANSAC iterations
		align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
		align.setCorrespondenceRandomness (5); // Number of nearest features to use
		align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
		align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
		align.setInlierFraction (0.90f); // Required inlier fraction for accepting a pose hypothesis
		{
		//	pcl::ScopeTime t("Alignment");
			align.align (*object_aligned);
		}

		float score = align.getFitnessScore();
		saveScore.push_back(score);
		if (score < mini)
		{
			mini = score;
            PointCloudT::Ptr temp(new PointCloudT);
            copyPointCloud(*object_aligned, *temp);
            *show = *temp;
		}
    }
        // vector<PointCloudT::Ptr> clouds;
        // clouds.push_back(cloud2);
        // clouds.push_back(show);
        // visualizeSlices(clouds);
	return mini;
}
// int main(int argc, char** argv)
// {
//     vector<string> models_filename;
//     string database_filename = "database";
//     models_filename.push_back("1_A.model");
//     models_filename.push_back("2_B.model");
//     models_filename.push_back("3_C.model");
//     models_filename.push_back("4_D.model");
//     models_filename.push_back("5_E.model");
//     models_filename.push_back("6_F.model");
//     models_filename.push_back("7_G.model");
//     models_filename.push_back("8_H.model");
//     models_filename.push_back("9_I.model");
//     stringstream ss;
//     int id;
//     ss << argv[1];
//     ss >> id;
//     Recog recog = Recog(models_filename, database_filename);
//     recog.search(id);
//     recog.info();
// }
