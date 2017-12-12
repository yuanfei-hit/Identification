#include "recog.h"

bool sortICPPair(pair<int, double> pair1, pair<int, double> pair2)
{
    return pair1.second < pair2.second;
}
bool sortID(pair<int, double> pair1, pair<int, double> pair2)
{
    return pair1.first < pair2.first;
}
bool sortPoint(PointT point1, PointT point2)
{
    return (point1.z < point2.z);
}
bool sortStiffness(float value_1, float value_2)
{
    return value_1 < value_2;
}


Recog::Recog()
{
	std::string path = ros::package::getPath("knowledge_base") + "/models/";
	vector<string> models_filename;
	models_filename.push_back(path + "1_A.model");
	models_filename.push_back(path + "2_B.model");
	models_filename.push_back(path + "3_C.model");
	models_filename.push_back(path + "4_D.model");
	models_filename.push_back(path + "5_E.model");
	models_filename.push_back(path + "6_F.model");
	models_filename.push_back(path + "7_G.model");
	models_filename.push_back(path + "8_H.model");
	models_filename.push_back(path + "9_I.model");
	for (int i = 0; i < models_filename.size(); i++)
			this->models.push_back(Model(models_filename[i]));
	this->database.readDatabase("database");
}
Recog::Recog(vector<string> models_filename, string database_filename)
{
    for (int i = 0; i < models_filename.size(); i++)
        this->models.push_back(Model(models_filename[i]));
    this->database.readDatabase(database_filename);
}
Recog::Recog(string database_filename)
{
	std::string path = ros::package::getPath("knowledge_base") + "/models/";
	vector<string> models_filename;
	models_filename.push_back(path + "1_A.model");
	models_filename.push_back(path + "2_B.model");
	models_filename.push_back(path + "3_C.model");
	models_filename.push_back(path + "4_D.model");
	models_filename.push_back(path + "5_E.model");
	models_filename.push_back(path + "6_F.model");
	models_filename.push_back(path + "7_G.model");
	models_filename.push_back(path + "8_H.model");
	models_filename.push_back(path + "9_I.model");
	for (int i = 0; i < models_filename.size(); i++)
	        this->models.push_back(Model(models_filename[i]));
	this->database.readDatabase(database_filename);
}
Recog::~Recog()
{

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

bool Recog::isRecognitionSuccess(int model_id, PointCloudT& visual, PointCloudNT& tactile_0, PointCloudNT& tactile_1, PointCloudNT& tactile_2, std::vector<float>& stiffness)
{
	bool flag = true;
	if (tactile_0.empty())
		flag = false;
	else
	{
		std::vector<std::pair<int, float> > stiffness_similarity_scores = getStiffnessSimilarityScores(model_id, stiffness);
		std::vector<std::pair<int, std::vector<std::pair<int, float> > > > slice_scores;
		int d_index = this->database.search(model_id);

		if (this->database.similar_stiffness_model_ids[d_index].empty())
		{
			for (int i = 0; i < stiffness_similarity_scores.size(); i ++)
				if ((stiffness_similarity_scores[0].second > stiffness_similarity_scores[i].second) || (stiffness_similarity_scores[0].second > 1.0))
					flag = false;
		}
		else
		{
			slice_scores = getSliceScores(model_id, visual, tactile_0, tactile_1, tactile_2);
			if ((stiffness_similarity_scores[0].second < 1.0))
			{
				for (int i = 0; i < slice_scores.size(); i ++)
				{
					for(int j = 0; j < slice_scores[i].second.size(); j ++)
					{
						if (slice_scores[i].second[0] > slice_scores[i].second[j])
							flag = false;
					}
				}
			}
			else
			{
				flag = false;
			}
		}
		/*std::cout << "Stiffness similarity scores" << std::endl;
		for (int i = 0; i < stiffness_similarity_scores.size(); i++)
			std::cout << "model_id: " << stiffness_similarity_scores[i].first << "; score: " << stiffness_similarity_scores[i].second << std::endl;*/

		std::cout << "Slice scores" << std::endl;
		for (int i = 0; i < slice_scores.size(); i++)
		{
			std::cout << "-- slice_id: " << slice_scores[i].first << std::endl;
			for (int j = 0; j < slice_scores[i].second.size(); j ++)
				std:: cout << "---- model_id: " << slice_scores[i].second[j].first << "; score: " << slice_scores[i].second[j].second << std::endl;
		}
	}
    return flag;
}

std::pair<float, Eigen::Matrix4f> Recog::getAlignScoreAndTF(PointCloudT::Ptr source, PointCloudT::Ptr target, float degree)
{
	int  times = 360.0/degree;
	float mini = 65535;
	std::vector<float> saveScore;
	Eigen::Matrix4f transformation;
    PointCloudT::Ptr show(new PointCloudT);

	for (int i = 0; i < times; i++)
	{
		PointCloudT::Ptr temp(new PointCloudT);
		Eigen::Affine3f transform (Eigen::Affine3f::Identity ());
		transform.rotate(Eigen::AngleAxisf(i*M_PI*degree/180.0, Eigen::Vector3f::UnitZ()));// rotate around Z axis
		pcl::transformPointCloud (*source, *temp, transform);

		PointCloudNT::Ptr object (new PointCloudNT);
		PointCloudNT::Ptr object_aligned (new PointCloudNT);
		PointCloudNT::Ptr scene (new PointCloudNT);
		FeatureCloudT::Ptr object_features (new FeatureCloudT);
		FeatureCloudT::Ptr scene_features (new FeatureCloudT);

		copyPointCloud (*temp, *object);
		copyPointCloud (*target, *scene);

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
		if (align.hasConverged ())
		{
			if (score < mini)
			{
				mini = score;
			//	std::cout << "mini_score: " << mini << std::endl;
				transformation = align.getFinalTransformation ();
				std::pair<float, Eigen::Matrix4f> score_tf = this->getAlignScoreAndTF(source, temp);
				transformation = transformation*score_tf.second;
			//	std::cout << transformation << std::endl;

				/*copyPointCloud(*object_aligned, *show);
				vector<PointCloudT::Ptr> clouds;
				clouds.push_back(target);
				clouds.push_back(show);
				clouds.push_back(source);
				visualizePointCloud(clouds);*/
			}
		}
    }
    /*vector<PointCloudT::Ptr> clouds;
    clouds.push_back(target);
    clouds.push_back(show);
    clouds.push_back(source);
    visualizePointCloud(clouds);*/
	return std::make_pair(mini, transformation);
}

std::pair<float, Eigen::Matrix4f> Recog::getAlignScoreAndTF(PointCloudT::Ptr source, PointCloudT::Ptr target)
{
	PointCloudNT::Ptr object (new PointCloudNT);
	PointCloudNT::Ptr object_aligned (new PointCloudNT);
	PointCloudNT::Ptr scene (new PointCloudNT);
	FeatureCloudT::Ptr object_features (new FeatureCloudT);
	FeatureCloudT::Ptr scene_features (new FeatureCloudT);

	copyPointCloud (*source, *object);
	copyPointCloud (*target, *scene);

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

	PointCloudT::Ptr show(new PointCloudT);
	Eigen::Matrix4f transformation;
	float score = align.getFitnessScore();
//	std::cout << "score: " << score << std::endl;
	if (align.hasConverged ())
	{
		transformation = align.getFinalTransformation ();
		/*std::cout << transformation << std::endl;
		copyPointCloud(*object_aligned, *show);
		vector<PointCloudT::Ptr> clouds;
		clouds.push_back(target);
		clouds.push_back(show);
		clouds.push_back(source);
		visualizePointCloud(clouds);*/
	}
	return std::make_pair(score, transformation);
}

// ##############################################
// Extract slice point cloud
// ##############################################
PointCloudT::Ptr Recog::extractSlicePointCloud(PointCloudT& cloud, float z_height)
{
	sort(cloud.points.begin(), cloud.points.end(), sortPoint);
	PointCloudT::Ptr slice (new PointCloudT);
	float epsilon = 0.0025;

	for (int i = 0; i < cloud.points.size(); i++)
	{
		if ((cloud.points[i].z >= (z_height - epsilon)) && (cloud.points[i].z <= (z_height + epsilon)))
		{
			slice->points.push_back(cloud.points[i]);
		}
		else if (cloud.points[i].z > (z_height + epsilon))
			break;
	}

	for (int i = 0; i < slice->points.size(); i++)
		slice->points[i].z = z_height;

	// Showing point cloud
	/*std::vector<PointCloudT::Ptr> show_cloud;
	PointCloudT::Ptr temp (new PointCloudT);
	*temp = cloud;
	show_cloud.push_back(temp);
	show_cloud.push_back(slice);
	visualizePointCloud(show_cloud);*/
	return slice;
}

std::vector<std::pair<int, float> > Recog::getStiffnessSimilarityScores(int model_id, std::vector<float>& stiffness)
{
	sort(stiffness.begin(), stiffness.end(), sortStiffness);
	std::vector<std::pair<int, float> > stiffness_similarity_scores;
	if (stiffness.size() > 1)
	{
		int counter = 0;
		float sum   = 0;
		float mean  = 0;
		for (int i = stiffness.size()/3; i < stiffness.size()*2/3; i ++)
	//	for (int i = 0; i < stiffness.size(); i ++)
		{
			sum += stiffness[i];
			counter ++;
		}
		mean = sum/counter;

		int m_index = getModelIndex(model_id);
		float score = fabs(mean - models[m_index].texture[0])/models[m_index].texture[1];

		stiffness_similarity_scores.push_back(std::make_pair(model_id, score));

		int d_index = this->database.search(model_id);
		for (int i = 0; i < this->database.similar_surface_model_ids[d_index].size(); i ++)
		{
			m_index = getModelIndex(this->database.similar_surface_model_ids[d_index][i]);
			score   = fabs(mean - models[m_index].texture[0])/models[m_index].texture[1];
			stiffness_similarity_scores.push_back(std::make_pair(this->database.similar_surface_model_ids[d_index][i], score));
		}

		std::cout << "Stiffness mean value: " << mean << std::endl;
		std::cout << "Stiffness similarity scores" << std::endl;
		for (int i = 0; i < stiffness_similarity_scores.size(); i++)
			std::cout << "-- model_id_" << stiffness_similarity_scores[i].first << "_score = " << stiffness_similarity_scores[i].second << std::endl;
	}
	else
	{
		std::cout << "stiffness size is not enough. Please check it" << std::endl;
	}

	return stiffness_similarity_scores;
}

std::vector<std::pair<int, std::vector<std::pair<int, float> > > > Recog::getSliceScores(int model_id, PointCloudT& visual, PointCloudNT& tactile_0, PointCloudNT& tactile_1, PointCloudNT& tactile_2)
{
	PointCloudT cloud_0;
	PointCloudT cloud_1;
    PointCloudT cloud_2;
    copyPointCloud (tactile_0, cloud_0);
    copyPointCloud (tactile_1, cloud_1);
    copyPointCloud (tactile_2, cloud_2);

    PointCloudT::Ptr source = extractSlicePointCloud(visual,  cloud_0.points[2].z);
    PointCloudT::Ptr target = extractSlicePointCloud(cloud_0, cloud_0.points[2].z);
    PointCloudT::Ptr sum (new PointCloudT);

    int num = (source->points.size() - target->points.size())/target->points.size();
	for (int j = 0; j < num; j ++)
		*sum += *target;
	*target = *sum + *target;

	std::pair<float, Eigen::Matrix4f> score_tranform = getAlignScoreAndTF(source, target, 5.0);
	std::cout << "transform from visual point cloud to contact point cloud" << std::endl;
	std::cout << score_tranform.second << std::endl;

	Eigen::Affine3f  transform;
	transform.matrix() = score_tranform.second;
	pcl::transformPointCloud (visual, visual, transform);

	std::vector<std::pair<int, std::vector<std::pair<int, float> > > > slice_scores;
	PointCloudT::Ptr visual_slice (new PointCloudT);
	PointCloudT::Ptr tactile_slice (new PointCloudT);
	PointCloudT::Ptr compaired_slice (new PointCloudT);
	PointCloudT::Ptr fused_slice (new PointCloudT);
	int d_index = this->database.search(model_id);
	for (int i = 0; i < this->database.slice_ids[d_index].size(); i ++)
	{
		int m_index = getModelIndex(model_id);
		if (this->database.slice_heights[d_index][i] < 0.11)
		{
			float p_index = getClosestPointIndex(cloud_2, this->database.slice_heights[d_index][i]);
			visual_slice    = extractSlicePointCloud(visual,  cloud_2.points[p_index].z);
			tactile_slice   = extractSlicePointCloud(cloud_2, cloud_2.points[p_index].z);
			compaired_slice = models[m_index].slices[this->database.slice_ids[d_index][i]];
			*fused_slice    = *visual_slice;
			int num = (compaired_slice->points.size() - visual_slice->points.size())/tactile_slice->points.size();
			if (num > 0)
				for (int j = 0; j < num; j ++)
					*fused_slice += *tactile_slice;
			else
				*fused_slice += *tactile_slice;

			ROS_WARN_STREAM("debug_0: num " << num);
			ROS_WARN_STREAM("debug_0: fused_slice size " << fused_slice->points.size());
			ROS_WARN_STREAM("debug_0: tactile_slice size " << tactile_slice->points.size());
			ROS_WARN_STREAM("debug_0: compaired_slice size " << compaired_slice->points.size());
		}
		else
		{
			float p_index = getClosestPointIndex(cloud_1, this->database.slice_heights[d_index][i]);
			visual_slice    = extractSlicePointCloud(visual,  cloud_1.points[p_index].z);
			tactile_slice   = extractSlicePointCloud(cloud_1, cloud_1.points[p_index].z);
			compaired_slice = models[m_index].slices[this->database.slice_ids[d_index][i]];
			*fused_slice    = *visual_slice;
			int num = (compaired_slice->points.size() - visual_slice->points.size())/tactile_slice->points.size();
			if (num > 0)
				for (int j = 0; j < num; j ++)
					*fused_slice += *tactile_slice;
			else
				*fused_slice += *tactile_slice;
		}
		score_tranform = getAlignScoreAndTF(compaired_slice, fused_slice, 5.0);
		std::vector<std::pair<int, float> > temp;
		temp.push_back(std::make_pair(model_id, score_tranform.first));

		for (int j = 0; j < this->database.dist_similar_surface_model_ids[d_index][i].size(); j ++)
		{
			m_index = getModelIndex(this->database.dist_similar_surface_model_ids[d_index][i][j]);
			if (this->database.slice_heights[d_index][i] < 0.11)
			{
				float p_index = getClosestPointIndex(cloud_2, this->database.slice_heights[d_index][i]);
				visual_slice    = extractSlicePointCloud(visual,  cloud_2.points[p_index].z);
				tactile_slice   = extractSlicePointCloud(cloud_2, cloud_2.points[p_index].z);
				compaired_slice = models[m_index].slices[this->database.slice_ids[d_index][i]];
				*fused_slice    = *visual_slice;
				int num = (compaired_slice->points.size() - visual_slice->points.size())/tactile_slice->points.size();
				if (num > 0)
					for (int j = 0; j < num; j ++)
						*fused_slice += *tactile_slice;
				else
					*fused_slice += *tactile_slice;
			}
			else
			{
				float p_index = getClosestPointIndex(cloud_1, this->database.slice_heights[d_index][i]);
				visual_slice    = extractSlicePointCloud(visual,  cloud_1.points[p_index].z);
				tactile_slice   = extractSlicePointCloud(cloud_1, cloud_1.points[p_index].z);
				compaired_slice = models[m_index].slices[this->database.slice_ids[d_index][i]];
				*fused_slice    = *visual_slice;
				int num = (compaired_slice->points.size() - visual_slice->points.size())/tactile_slice->points.size();
				if (num > 0)
					for (int j = 0; j < num; j ++)
						*fused_slice += *tactile_slice;
				else
					*fused_slice += *tactile_slice;
			}
			score_tranform = getAlignScoreAndTF(compaired_slice, fused_slice, 5.0);
			temp.push_back(std::make_pair(this->database.dist_similar_surface_model_ids[d_index][i][j], score_tranform.first));
		}
		slice_scores.push_back(std::make_pair(this->database.slice_ids[d_index][i], temp));
	}
	return slice_scores;
}

int Recog::getClosestPointIndex(PointCloudT& cloud, float z_height)
{
	int point_index = 0;
	float min_value = fabs(z_height - cloud.points[0].z);

	for (int i = 0; i < cloud.points.size(); i ++)
	{
		if (min_value > fabs(z_height - cloud.points[i].z))
		{
			min_value = fabs(z_height - cloud.points[i].z);
			point_index = i;
		}
	}
	return point_index;
}

void Recog::visualizePointCloud(std::vector<PointCloudT::Ptr> cloud)
{
   RGB rgbs[] = {
            RGB(255,   0,   0),
            RGB(255, 165,   0),
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
	for (int i = 0; i < cloud.size(); i++)
	{
		ColorHandlerT handle (cloud[i], rgbs[i%12].getR(), rgbs[i%12].getG(), rgbs[i%12].getB());
		stringstream ss;
		string index;
		ss << i;
		ss >> index;
		viewer.addPointCloud (cloud[i], handle, "cloud_visualization"+index);
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_visualization"+index);
	}
	viewer.addCoordinateSystem (0.1, 0, 0, 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
}

void Recog::visualizePointCloud(std::vector<PointCloudNT::Ptr> cloud)
{
   RGB rgbs[] = {
            RGB(255,   0,   0),
            RGB(255, 165,   0),
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
	for (int i = 0; i < cloud.size(); i++)
	{
		ColorHandlerNT handle (cloud[i], rgbs[i%12].getR(), rgbs[i%12].getG(), rgbs[i%12].getB());
		stringstream ss;
		string index;
		ss << i;
		ss >> index;
		viewer.addPointCloud (cloud[i], handle, "cloud_visualization"+index);
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_visualization"+index);
	}
	viewer.addCoordinateSystem (0.1, 0, 0, 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
}
 /*int main(int argc, char** argv)
 {
     vector<string> models_filename;
     string database_filename = "database";
     models_filename.push_back("1_A.model");
     models_filename.push_back("2_B.model");
     models_filename.push_back("3_C.model");
     models_filename.push_back("4_D.model");
     models_filename.push_back("5_E.model");
     models_filename.push_back("6_F.model");
     models_filename.push_back("7_G.model");
     models_filename.push_back("8_H.model");
     models_filename.push_back("9_I.model");
     stringstream ss;
     int id;
     ss << argv[1];
     ss >> id;
     Recog recog = Recog(models_filename, database_filename);
     recog.search(id);
     recog.info();
 }*/
