#include "model_builder.h"

void saveMatrix(string filename, Eigen::MatrixXf matrix)
{
	ofstream file;
    file.open(filename.c_str());
    for (int i = 0; i < matrix.rows(); i++)
    {
        for (int j = 0; j < matrix.cols(); j++)
		{
            file << matrix(i, j) << " ";
        }
        file << "\n";
    }
    file.close();
}

bool sortVector(int value1, int value2)
{
	return value1 < value2;
}

void ModelBuilder::saveModelPairDatabase(string filename)
{
    ofstream file;
    file.open(filename.c_str());
    file << "#Model Pair Database File Format\n";

    map<pair<int, int>, pair<bool, vector<pair<int, float> > > >::iterator it;
    for (it = this->model_pair_database.begin(); it != this->model_pair_database.end(); ++it)
    {
    	file << "MODEL_PAIR_IDS " << it->first.first << " " << it->first.second<< "\n";
    	if (it->second.first == true)
    		file << "similarity " << 1<< "\n";
    	else
    		file << "similarity " << 0<< "\n";
    	    file << "slice_scores ";
    	for(int i = 0; i < it->second.second.size(); i++)
    		file << it->second.second[i].second << " ";
    	file << "\n";
    	file << "END" << "\n";
    }
    file << "FILEEND\n";
}

void ModelBuilder::readModelPairDatabase(string filename)
{
    fstream file;
    file.open(filename.c_str());
    string line;
    bool similarity_flag = false;
    pair<int, int> ids;
	vector<pair<int, float> > slice_scores;
    while(getline(file, line))
    {
    	if (line == "FILEEND")
    		break;
    	if (line.find("#") != -1)
            continue;

    	stringstream ss(line);
    	string value;
    	vector<string> values;
    	while(ss >> value)
    	{
    		values.push_back(value);
    	}
    	if (values[0] == "MODEL_PAIR_IDS")
        {
        	stringstream ss1, ss2;
        	ss1 << values[1];
        	ss2 << values[2];
        	ss1 >> ids.first;
        	ss2 >> ids.second;
        }
    	else if(values[0] == "similarity")
        {
    		stringstream ss1;
    		ss1 << values[1];
    		int temp;
    		ss1 >> temp;
    		if (temp == 1)
    			similarity_flag = true;
    		else
    			similarity_flag = false;
        }
    	else if(values[0] == "slice_scores")
    	{
			for (int i=1; i<values.size(); i++)
			{
				stringstream ss1;
				ss1 << values[i];
				float score;
				ss1 >> score;
				slice_scores.push_back(std::make_pair(i-1, score));
			}
    	}
    	else if (values[0] == "END")
		{
			this->model_pair_database[ids].first = similarity_flag;
			this->model_pair_database[ids].second = slice_scores;
			ids.first  = -1;
			ids.second = -1;
			similarity_flag = false;
			slice_scores.clear();
		}
    }
}

ModelBuilder::ModelBuilder(vector<string> filenames)
{
    this->model_pair_database_filename = "model_pair_database.txt";
	for (int i = 0; i < filenames.size(); i++)
    {
        Model model = Model(filenames[i]);
        this->models.push_back(model);
    }
  //  buildModelPairDatabase();
    readModelPairDatabase(this->model_pair_database_filename);
  //  outputModelPairInfo();
    buildSimilarModelDatabase();
    printf("| model size: %d | \n", (int)this->models.size());
}

void ModelBuilder::buildModelPairDatabase()
{
    Eigen::MatrixXf matrix_similar (this->models.size(), this->models.size());
    for (int i = 0; i < this->models.size(); i ++)
        for (int j = 0; j < this->models.size(); j++)
        	matrix_similar(i, j) = 0.0;

    for (int i = 0; i < this->models.size(); i ++)
    {
        for (int j = i+1; j < this->models.size(); j ++)
        {
            // printf("Comparing model %d and %d \n", this->models[i].id, this->models[j].id);
            bool similar = isSimilar(this->models[i], this->models[j]);
            vector<pair<int, float> > slice_score_vector;

            if (similar == true)
            {
                matrix_similar(i, j) = 1.0;
                int slice_num = min(this->models[i].slice_num, this->models[j].slice_num);

                Eigen::MatrixXf matrix(1, slice_num);

                for (int k=0; k<slice_num; k++)
                {
                    PointCloudT::Ptr slice1(new PointCloudT);
                    PointCloudT::Ptr slice2(new PointCloudT);

                    *slice1 = *this->models[i].slices[k];
                    *slice2 = *this->models[j].slices[k];

                    float score1 = getAlignScore(slice1, slice2, 5.0);
                    float score2 = getAlignScore(slice2, slice1, 5.0);

                    float score = min(score1, score2);

                    slice_score_vector.push_back(std::make_pair(k, score));

                    matrix(k, 0) = score;
                }

			    /*stringstream ss1, ss2;
				string id1, id2;
				ss1 << this->models[i].id;
				ss1 >> id1;
				ss2 << this->models[j].id;
				ss2 >> id2;
				saveMatrix("models_id_"+id1+"_"+id2+"_slice_scores.txt", matrix);*/
            }

            pair<bool, vector<pair<int, float> > > temp(similar, slice_score_vector);
            pair<int, int> ids(this->models[i].id, this->models[j].id);
            this->model_pair_database[ids] = temp;
        }
    }

    //saveModelPairDatabase(this->model_pair_database_filename);
    //saveMatrix("matrix_similar_models.txt", matrix_similar);

    outputModelPairInfo();
}

void ModelBuilder::buildSimilarModelDatabase()
{
	map<int, pair<vector<int>, vector<vector<pair<int, float> > > > > similar_model_slice_scores = getSliceScoresForSimilarModel();
	map<int, pair<vector<int>, vector<vector<pair<int, float> > > > >::iterator it;
	Database similar_model_database;

	for (it = similar_model_slice_scores.begin(); it != similar_model_slice_scores.end(); ++it)
	{
		//Determine the matrix size
		int row = it->second.second[0].size();
		int column = it->second.second.size();
		for (int i=0; i< column; i++)
		{
			if (row < it->second.second[i].size())
				row = it->second.second[i].size();
		}

		//Set the matrix
		Eigen::MatrixXf matrix (row, column);
		for (int i=0; i< row; i++)
			for (int j=0; j< column; j++)
				matrix(i, j) = it->second.second[j][i].second;

		/*printf("************************************ \n");
		cout << matrix << endl;
		printf("************************************ \n");*/

		//Set model index
		int  model_index;
		bool model_index_valid_flag = false;
		for (int i = 0; i < this->models.size(); i++)
		{
			if (it->first == this->models[i].id)
			{
				model_index = i;
				model_index_valid_flag = true;
			}
		}

		if (model_index_valid_flag == true)
		{
			//Search slices without considering trimmed slices to find which slices can distinguish each similar model from target model
			set<int> trimmed_slice_index;
			for (int i = 0; i < this->models[model_index].trimmed_slice_index.size(); i++)
				trimmed_slice_index.insert(this->models[model_index].trimmed_slice_index[i]);

			vector<vector<int> >   slice_index;
			vector<vector<float> > slice_score;
			float slice_score_threshold = 3e-5;
			for (int i = 0; i < column; i++)
			{
				set<int>      trimmed_slice_index_temp = trimmed_slice_index;
				vector<int>   slice_index_temp;
				vector<float> slice_score_temp;
				for (int j = 0; j < row; j++)
				{
					slice_score_temp.push_back((float)matrix(j, i));
					int size = trimmed_slice_index_temp.size();
					trimmed_slice_index_temp.insert(j);
					if (size != trimmed_slice_index_temp.size())
						if (matrix(j, i) > slice_score_threshold)
							slice_index_temp.push_back(j);
				}
				slice_index.push_back(slice_index_temp);
				slice_score.push_back(slice_score_temp);
			}

			/*printf("************************************ \n");
			printf("Model id:  %d\n", it->first);
			for (int i = 0; i < slice_index.size(); i++)
			{
				printf("found rows of column %d: ", i);
				for (int j = 0; j < slice_index[i].size(); j++)
					printf("%d ", slice_index[i][j]);
				printf("\n");
			}
			printf("************************************ \n");*/

			vector<int> similar_sli_model_ids;
			vector<int> similar_sti_model_ids;
			vector<int> slice_ids;
			vector<float> slice_heights;
			vector<vector<int> > dist_similar_sur_model_ids;

			//Set similar slice model ids
			for (int i = 0; i < slice_index.size(); i++)
				if (slice_index[i].empty())
					similar_sli_model_ids.push_back(it->second.first[i]);

			//Set similar stiffness model ids
			pair<float, float> stiffness = make_pair(this->models[model_index].texture[0], this->models[model_index].texture[1]);
			for (int i = 0; i < it->second.first.size(); i ++)
			{
				int index = -1;
				for (int j = 0; j < this->models.size(); j++)
					if (it->second.first[i] == this->models[j].id)
						index = j;
				float delta = fabs(this->models[index].texture[0] - stiffness.first);

				if(index == -1)    std::cout << "Cann't find the model id " << it->second.first[i];

				if ((delta < this->models[index].texture[1]) && (delta < stiffness.second))
					similar_sti_model_ids.push_back(it->second.first[i]);
			}

			//Set optimal slice ids
			slice_ids = getOptimalSliceID(slice_index, slice_score, slice_score_threshold, 10000);

			//Set slice heights and distinguished similar surface model ids
			for (int i = 0; i < slice_ids.size(); i ++)
			{
				slice_heights.push_back(0.5*(this->models[model_index].spans[slice_ids[i]].first + this->models[model_index].spans[slice_ids[i]].second));

				vector<int> temp;
				for (int j = 0; j < column; j ++)
					if (matrix(slice_ids[i], j) > slice_score_threshold)
						temp.push_back(it->second.first[j]);
				dist_similar_sur_model_ids.push_back(temp);
			}

			similar_model_database.insert(it->first, it->second.first, similar_sli_model_ids, similar_sti_model_ids, slice_ids, slice_heights, dist_similar_sur_model_ids);
		}
	}
	similar_model_database.info();
    printf("database size: %d \n", similar_model_database.size());
    similar_model_database.saveDatabase("database");
}

map<int, pair<vector<int>, vector<vector<pair<int, float> > > > > ModelBuilder::getSliceScoresForSimilarModel()
{
    map<int, pair<vector<int>, vector<vector<pair<int, float> > > > > similar_model_slice_scores;
    map<pair<int, int>, pair<bool, vector<pair<int, float> > > >::iterator it;

    set<int> model_ids;
    for (it = this->model_pair_database.begin(); it != this->model_pair_database.end(); ++it)
    {
    	model_ids.insert(it->first.first);
    	model_ids.insert(it->first.second);
    }

    set<int>::iterator model_ids_it;
    for (model_ids_it = model_ids.begin() ; model_ids_it != model_ids.end() ; ++model_ids_it)
    {
    	vector<int> similar_model_ids;
    	vector<vector<pair<int, float> > > temp;
    	int id = *model_ids_it;

    	for (it = this->model_pair_database.begin(); it != this->model_pair_database.end(); ++it)
		{
			if (it->second.first == true)
			{
				if (it->first.second == id)
				{
					similar_model_ids.push_back(it->first.first);
					temp.push_back(it->second.second);
				}
				else if(it->first.first == id)
				{
					similar_model_ids.push_back(it->first.second);
					temp.push_back(it->second.second);
				}
			}
		}

    	similar_model_slice_scores[id] = std::make_pair(similar_model_ids, temp);

    	/*//Determine the matrix size
		int row = similar_model_slice_scores[id].second[0].size();
		int column = similar_model_slice_scores[id].second.size();
		for (int i=0; i< column; i++)
		{
			if (row < similar_model_slice_scores[id].second[i].size())
				row = similar_model_slice_scores[id].second[i].size();
		}

		//Set the matrix
		Eigen::MatrixXf matrix (row, column);
		for (int i=0; i< row; i++)
			for (int j=0; j< column; j++)
				matrix(i, j) = similar_model_slice_scores[id].second[j][i].second;

		stringstream ss;
		string s_id;
		ss << id;
		ss >> s_id;
		saveMatrix("slice_score_between_similar_models_and_model_id_" + s_id + ".txt", matrix);*/
    }

    return similar_model_slice_scores;
}

bool ModelBuilder::isSimilar(Model model1, Model model2)
{
    // for (int i < )
    // return true;
    // printf("----------- Similarity -------------\n");
    // printf("model1: %f , model2: %f \n", model1.height, model2.height);
    // printf("------------------------\n");
    if (abs(model1.height - model2.height) > 0.03)
        return false;

    float score_threshold = 20e-5;//1.5e-5;
    float rate_threshold = 0.1;

    /*Eigen::MatrixXf matrix1(model1.raw_num, model2.raw_num);
    Eigen::MatrixXf matrix2(model1.raw_num, model2.raw_num);
    Eigen::MatrixXf matrix3(model1.raw_num, model2.raw_num);
    Eigen::MatrixXf matrix4(model1.raw_num, model2.raw_num);
    Eigen::MatrixXf matrix5(model2.raw_num, model1.raw_num);
    Eigen::MatrixXf matrix6(model2.raw_num, model1.raw_num);*/
    for (int i = 0; i < model1.raw.size(); i++)
    {
        for (int j = 0; j < model2.raw.size(); j++)
        {
        	float score1 = getAlignScore(model1.raw[i], model2.raw[j], 360.0);
            float score2 = getAlignScore(model2.raw[j], model1.raw[i], 360.0);
            float score;
            float rate1 = fabs((float) model1.raw[i]->points.size()/model2.raw[j]->points.size() - 1);
            float rate2 = fabs((float) model2.raw[j]->points.size()/model1.raw[i]->points.size() - 1);
            float rate;
            if (score1 < score2)
            {
                score = score1;
                rate  = rate1;
            }
            else
            {
                score = score2;
                rate  = rate2;
            }
            printf("model_%d_raw: %d , model_%d_raw: %d , score: %f , rate: %f \n", model1.id, i, model2.id, j, score, rate);

            if (score <= score_threshold && rate <= rate_threshold)
            {
                return true;
            }
             /*matrix1(i, j) = score;
             matrix2(i, j) = rate;
             matrix3(i, j) = score1;
             matrix4(i, j) = rate1;
             matrix5(j, i) = score2;
             matrix6(j, i) = rate2;*/
        }
    }
   /* stringstream ss1, ss2;
    string id1, id2;
    ss1 << model1.id;
    ss1 >> id1;
    ss2 << model2.id;
    ss2 >> id2;
    saveMatrix("matrix_raw_score_id_"+id1+"-"+id2+".txt", matrix1);
    saveMatrix("matrix_raw_rate_id_"+id1+"-"+id2+".txt", matrix2);
    saveMatrix("matrix_raw_score_id_"+id1+id2+".txt", matrix3);
    saveMatrix("matrix_raw_rate_id_"+id1+id2+".txt", matrix4);
    saveMatrix("matrix_raw_score_id_"+id2+id1+".txt", matrix5);
    saveMatrix("matrix_raw_rate_id_"+id2+id1+".txt", matrix6);*/
    return false;
}

vector<int> ModelBuilder::getOptimalSliceID(vector<vector<int> > slice_index, vector<vector<float> > slice_score, float slice_score_threshold, int run_times)
{
	int min_size = 65535;
	int counter = 0;
	vector<int> res;
	//srand(12345);
	srand(time(NULL));

	for (int counter = 0; counter < run_times; counter ++)
	{
		vector<int> index;
		for (int i = 0; i < slice_index.size(); i ++)
		{
			if (slice_index[i].empty() == false)
			{
				index.push_back(slice_index[i][rand() % slice_index[i].size()]);
				for (int j = i+1; j < slice_index.size(); j ++)
				{
					if (slice_index[j].empty() == false)
					{
						bool flag = false;
						for (int k = 0; k < slice_index[j].size(); k ++)
						{
							vector<int>::iterator iter;
							for (iter = index.begin(); iter != index.end(); ++ iter)
							{
								if (*iter == slice_index[j][k])
								{
									flag = true;
							        break;
								}
							}
							if (flag == true)
								break;
						}
						if (flag == false)
							index.push_back(slice_index[j][rand() % slice_index[j].size()]);
					}
				}
				break;
			}
		}
		sort(index.begin(), index.end(), sortVector);

		int size = index.size();
		if (size < min_size)
		{
			min_size = size;
			res = index;
		}
		else if (size == min_size)
		{
			float sum_new = 0.0;
			float sum_old = 0.0;
			int   counter_new = 0;
			int   counter_old = 0;

			for (int i = 0; i < index.size(); i ++)
			{
				for (int j = 0; j < slice_score.size(); j ++)
				{
					if (slice_score[j][index[i]] > slice_score_threshold)
					{
						sum_new += slice_score[j][index[i]];
						counter_new ++;
					}
				}
			}

			for (int i = 0; i < res.size(); i ++)
			{
				for (int j = 0; j < slice_score.size(); j ++)
				{
					if (slice_score[j][res[i]] > slice_score_threshold)
					{
						sum_old += slice_score[j][res[i]];
						counter_old ++;
					}
				}
			}

			if (((float)sum_new/counter_new) > ((float)sum_old/counter_old))
			{
				res = index;
			//	printf("%f\n", sum_new/counter_new);
			}
		}
	}
	/*printf("-------------------\n");
	printf("optimal slice id: ");
	for (int i = 0; i < res.size(); i ++)
		printf("%d ", res[i]);
	printf("\n");
	printf("-------------------\n");*/

	return res;
}

float ModelBuilder::getAlignScore(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2, float degree)
{
	int times = 360.0/degree;
	PointCloudT::Ptr temp(new PointCloudT);
	*temp = *cloud1;
	float mini = 65535;
	vector<float> saveScore;
	//Eigen::Matrix4f transformation;
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

		// Downsample
	//	pcl::console::print_highlight ("Downsampling...\n");
		pcl::VoxelGrid<PointNT> grid;
		const float leaf = 0.005f;
		/*grid.setLeafSize (leaf, leaf, leaf);
		grid.setInputCloud (object);
		grid.filter (*object);
		grid.setInputCloud (scene);
		grid.filter (*scene);*/

		// Estimate normals for scene
	//	pcl::console::print_highlight ("Estimating scene normals...\n");
		pcl::NormalEstimationOMP<PointNT,PointNT> nest;
		nest.setRadiusSearch (0.02);
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
		align.setInlierFraction (0.70f); // Required inlier fraction for accepting a pose hypothesis
		{
//			pcl::ScopeTime t("Alignment");
			align.align (*object_aligned);
		}

		float score = align.getFitnessScore();
		saveScore.push_back(score);
		if (score < mini)
		{
			mini = score;
		}

		/*if (align.hasConverged ())
			pcl::console::print_info ("Inliers: %i/%i; score: %f\n", align.getInliers ().size (), object->size (), mini);
		else
			pcl::console::print_error ("Alignment failed!\n");

		vector<PointCloudNT::Ptr> show_cloud;
		show_cloud.push_back(object);
		show_cloud.push_back(object_aligned);
		show_cloud.push_back(scene);
		visualizePointCloud(show_cloud);*/
	}
	/*writeFile("saveScore.txt", saveScore);
	vector<PointCloudT::Ptr> show_cloud;
	show_cloud.push_back(cloud1);
	show_cloud.push_back(cloud2);
	visualizePointCloud(show_cloud);*/
	return mini;
}

void ModelBuilder::outputModelPairInfo()
{
    printf("**********     MODEL BUILDING INFO     ********** \n");
    map<pair<int, int>, pair<bool, vector<pair<int, float> > > >::iterator it;
    for (it = this->model_pair_database.begin(); it != this->model_pair_database.end(); ++it)
    {
        printf("[   IDs   | %d  |    %d ] \n", it->first.first, it->first.second);
        if (it->second.first == true)
            printf("[ Similar Objects ] \n");
        else
            printf("[ Different Objects ] \n");
        printf("[ Slice Index | ICP Score ] \n");
        for (int i = 0; i < it->second.second.size(); i++)
        {
        printf("[ %d | %f ] \n", it->second.second[i].first, it->second.second[i].second);
        }
        printf("************************************ \n");
    }
}

void ModelBuilder::rotatePointCloud(PointCloudT::Ptr &slice, float degree, char mode)
{
	Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
	float angle = M_PI*degree/180.0;
    if (mode == 'z')
    {
        matrix(0,0) = cos(angle);
        matrix(0,1) = -sin(angle);
        matrix(1,0) = sin(angle);
        matrix(1,1) = cos(angle);
    }
    else if (mode == 'y')
    {
        matrix(0,0) = cos(angle);
        matrix(0,2) = -sin(angle);
        matrix(2,0) = sin(angle);
        matrix(2,2) = cos(angle);
    }
    else if (mode == 'x')
    {
        matrix(1,1) = cos(angle);
        matrix(1,2) = -sin(angle);
        matrix(2,1) = sin(angle);
        matrix(2,2) = cos(angle);
    }
    else
    {
        cout << "insert true mode index !" << endl;
    }
	pcl::transformPointCloud(*slice, *slice, matrix);
}

void ModelBuilder::visualizePointCloud ( vector<PointCloudT::Ptr> cloud, vector<RGB> rgb)
{
	pcl::visualization::PCLVisualizer viewer("cloud");
    for (int i = 0; i < cloud.size(); i++)
    {
        ColorHandlerT handle (cloud[i], rgb[i].getR(), rgb[i].getG(), rgb[i].getB());
        stringstream ss;
        string str;
        ss << i;
        str = "cloud_"+ss.str();
        viewer.addPointCloud (cloud[i], handle, str);
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, str);
    }
    viewer.addCoordinateSystem (0.1, 0, 0, 0);
	viewer.setBackgroundColor(0., 0., 0., 0);
	// viewer.initCameraParameters ();
  	while (!viewer.wasStopped ())
  	{
  		viewer.spinOnce ();
  	}
}

void ModelBuilder::visualizePointCloud(vector<PointCloudNT::Ptr> cloud)
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
		viewer.addPointCloud (cloud[i], handle, "cloud"+index);
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud"+index);
	}
	viewer.addCoordinateSystem (0.1, 0, 0, 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
}

void ModelBuilder::visualizePointCloud ( PointCloudT::Ptr cloud, RGB rgb )
{
	pcl::visualization::PCLVisualizer viewer("cloud");
	ColorHandlerT handle (cloud, rgb.getR(), rgb.getG(), rgb.getB());
	viewer.addPointCloud (cloud, handle, "cloud_visualization");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_visualization");
	// viewer.addCoordinateSystem (0.2, "sensor", 0);
    viewer.addCoordinateSystem (0.1, 0, 0, 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
	// viewer.initCameraParameters ();
  	while (!viewer.wasStopped ())
  	{
  		viewer.spinOnce ();
  	}
}

Tree::Tree()
{
    this->head = NULL;
}
void Tree::ins(Slice data)
{
    Node* r = new Node(data);
    Node* location = NULL;
    bool isExist = false;
        // printf("[ insert node ]: id: %d\n", data.id);
        // printf("[ %f, %f ] \n", data.span.first, data.span.second);
        // printf("------------------------------------\n");
    if (this->head == NULL)
    {
        this->head = r;
    }
    else
    {
        location = search(data.span, isExist);
        if (isExist)
        {
            location->slice.span.first = max(location->slice.span.first, data.span.first);
            location->slice.span.second = min(location->slice.span.second, data.span.second);
            location->height = (location->slice.span.first + location->slice.span.second) * 0.5;
            location->slice.height = location->height;
            location->ids.push_back(data.id);
        }
        else
        {
            r->father = location;
            if (r->slice.span.second < location->slice.span.first)
                location->left = r;
            else
                location->right = r;
        }
    }
    // if (location != NULL)
    // {
    // print(this->head);
    // printf("is exist: %d \n", isExist);
    // printf("[ r span: %f %f ]\n", r->slice.span.first, r->slice.span.second);
    // printf("[ location span: %f %f ]\n", location->slice.span.first, location->slice.span.second);
    // printf("--------------------------------\n");
    // printf("|                              |\n");
    // printf("|                              |\n");
    // printf("|                              |\n");
    // printf("--------------------------------\n");
    // }
}
Node* Tree::search(pair<float, float> span, bool& isExist)
{
    Node* p = NULL;
    p = this->head;
    Node* q = NULL;
    float epsilon = 1e-6;
    while(p)
    {
        q = p;
        if (!((span.second < p->slice.span.first) || (span.first > p->slice.span.second)))
        {
            isExist = true;
            return p;
        }
        if (span.first > p->slice.span.second)
            p = p->right;
        else
            p = p->left;
    }
    isExist = false;
    return q;
}
Node* Tree::search(float height, bool& isExist)
{
    Node* p = NULL;
    p = this->head;
    Node* q = NULL;
    float epsilon = 1e-6;
    while(p)
    {
        q = p;
        if (p->height >= height - epsilon && p->height <= height + epsilon)
        {
            isExist = true;
            return p;
        }
        if (p->height < height)
            p = p->right;
        else
            p = p->left;
    }
    isExist = false;
    return q;
}
void Tree::print(Node* node)
{
    if (node == NULL)
        return;
    printf("[ ids: ");
    for (int i = 0; i < node->ids.size(); i++)
        printf("%d ", node->ids[i]);
    printf(", span: %f - %f \n", node->slice.span.first, node->slice.span.second);
    if (node->left!=NULL)
        print(node->left);
    if (node->right!=NULL)
        print(node->right);
}
vector<float> Tree::getValues()
{
    vector<float> heights;
    stack<Node*> s;
    Node* p = this->head;
    while(p != NULL || !s.empty())
    {
        while(p != NULL)
        {
            s.push(p);
            p = p->left;
        }
        if (!s.empty())
        {
            p = s.top();
            heights.push_back(p->height);
            s.pop();
            p = p->right;
        }
    }
    return heights;
}
void Tree::getValues(vector<float> &height, vector<pair<float, float> > &span, vector<vector<int> > &ids)
{
    height.clear();
    span.clear();
    ids.clear();
    stack<Node*> s;
    Node* p = this->head;
    while(p != NULL || !s.empty())
    {
        while(p != NULL)
        {
            s.push(p);
            p = p->left;
        }
        if (!s.empty())
        {
            p = s.top();
            height.push_back(p->height);
            span.push_back(p->slice.span);
            ids.push_back(p->ids);
            s.pop();
            p = p->right;
        }
    }
}
