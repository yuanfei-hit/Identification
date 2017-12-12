// #pragma once
// #include "head.h"
// #include "properties.h"
#include "extract_feature.h"


bool sortPointCloud(PointT point1, PointT point2)
{
    return (point1.z < point2.z);
}

bool sortMap(pair<float, vector<PointT> > map1, pair<float, vector<PointT> > map2)
{
	return (map1.second.size() > map2.second.size());
}

bool sortSlice(PointCloudT::Ptr slice1, PointCloudT::Ptr slice2)
{
	return slice1->points[0].z > slice2->points[0].z;
}

// ##############################################
// Constructor
// ##############################################
FeatureExtraction::FeatureExtraction(int id, string filename, string config)
{
	this->id = id;
    this->filename = filename;
    this->config = config;
	this->cloud = PointCloudT::Ptr (new PointCloudT);
	// this->PI = 3.1415926535897932384626;
	if (this->filename.find(".pcd")==-1)
		this->name = this->filename;
	else
		this->name = string(this->filename.substr(0, filename.find(".pcd")));

    printf("______________________________________________\n");
    printf("|          FEATURE CONSTRUCTOR INFO          |\n");
    printf("| id:               %d\n", this->id);
    printf("| name:             %s\n", this->name.c_str());
    printf("| config:           %s\n", this->config.c_str());
    printf("| filename:         %s\n", this->filename.c_str());
    printf("______________________________________________\n");
    readConfig();
    loadPointCloud();
    extractSlices();
	visualizeSlices(this->slices);
//	savePointCloud(this->slices, "whole");
//	savePointCloud(this->extracted_slices, "fraction");
//	extractKeyPoints();
//	savePointCloud(this->key_points, "key");
//	 writeFile("slices_span.txt", this->slices_span);

}

// ##############################################
// Default Constructor
// ##############################################
FeatureExtraction::FeatureExtraction()
{
}
FeatureExtraction::~FeatureExtraction()
{

}
// ##############################################
// Read PCD file
// ##############################################
void FeatureExtraction::loadPointCloud()
{
	pcl::PCDReader reader;
	reader.read<PointT> ( this->filename, *this->cloud );
}

// ##############################################
// Extract slices
// ##############################################
void FeatureExtraction::extractSlices()
{
	vector<int> indices;
	pcl::removeNaNFromPointCloud(*this->cloud, *this->cloud, indices);
	sort(this->cloud->points.begin(), this->cloud->points.end(), sortPointCloud);
	this->height = this->cloud->points[this->cloud->points.size()-1].z;
	int index = 0;
	int number = 0;
	for (int i = 0; i < this->slices_size; i++)
	{
		this->search_base += this->interval;
		map<float, vector<PointT> > mapping;
		bool flag = false;
		PointCloudT::Ptr slice(new PointCloudT);
		while(1)
		{
			if (this->cloud->points[index].z > this->search_base - this->epsilon && this->cloud->points[index].z < this->search_base + this->epsilon)
			{
				mapping[this->cloud->points[index].z].push_back(this->cloud->points[index]);
				flag = true;
			}
			else
			{
				if (flag == true)
					break;
			}
			index++;
			if (index >=this->cloud->points.size())
			{
				break;
			}
		}
		// sort(mapping.begin(), mapping.end(), sortMap);
		map<float, vector<PointT> >::iterator it;
		for (it = mapping.begin(); it!= mapping.end(); ++it)
		{
			float key = it->first;
			for (int j = 0; j < mapping[key].size(); j++)
			{
				PointT point_temp = mapping[key][j];
				point_temp.z = this->search_base;
				slice->push_back(point_temp);
			}
			if (slice->points.size() > this->search_num)
			{
				break;
			}
		}	
		if (slice->points.size() != 0 && slice->points.size() >= this->slice_points )
		{
			this->slices.push_back(slice);
			number ++ ;
		}
		if (index >= this->cloud->points.size())
		{
			if (i < this->slices_size)
			{
                printf("[WARNING] Can\'t get %d slices. \n", this->slices_size);
                printf("[INFO] The exact slice number is %d \n", number);
			}
			break;
		}
	}
	selectSlices();
}
void FeatureExtraction::readConfig()
{
	SliceProperties properties(this->config);
	this->slices_size = properties.slices_size;
	this->interval = properties.interval;
	this->epsilon = properties.epsilon;
	this->search_base = properties.search_base;
	this->search_num = properties.search_num;
	this->base = this->search_base;
	this->threshold = properties.threshold;
	this->slice_points = properties.slice_points;
	this->valid_height = properties.valid_height;
	this->order = properties.order;
	properties.info();
}

void FeatureExtraction::savePointCloud(vector<PointCloudT::Ptr> clouds, string prefix)
{
	// int index = this->filename.find_last_of("_")+1;
	for (int i = 0; i < clouds.size(); i++)
	{
		stringstream ss;
		string temp;
		ss << i;
		ss >> temp;
		pcl::io::savePCDFileASCII(prefix+"_slice_"+temp+".pcd", *clouds[i]);
	}
}
void FeatureExtraction::visualizeSlices(vector<PointCloudT::Ptr> visual)
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

void FeatureExtraction::visualizeSlices(vector<PointCloudNT::Ptr> visual)
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
	for (int i = 0; i < visual.size(); i++)
	{
		ColorHandlerNT handle (visual[i], rgbs[i%12].getR(), rgbs[i%12].getG(), rgbs[i%12].getB());
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
float FeatureExtraction::getICPScore(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2, float degree)
{
	int times = 360.0/degree;
	PointCloudT::Ptr temp(new PointCloudT);
	*temp = *cloud1;
	float mini = 65535;
	vector<float> saveScore;
	// Eigen::Matrix4f transformation;
	for (int i = 0; i < times; i++)
	{
		rotatePointCloud(temp, degree, 'z');
		pcl::IterativeClosestPoint<PointT, PointT> icp;
		Eigen::Matrix4f matrix;
		icp.setInputSource(temp);
		icp.setInputTarget(cloud2);
		PointCloudT align;
		PointCloudT::Ptr final(new PointCloudT);
		icp.align(align);
		*final = align;
		float score = icp.getFitnessScore();
		saveScore.push_back(score);
		if (score < mini)
		{
			mini = score;
			// transformation = icp.getFinalTransformation();
		}
/*		if (i == 9)
		{
			pcl::io::savePCDFile("slice1.pcd", *temp);
			pcl::io::savePCDFile("slice2.pcd", *cloud2);
		}
		vector<PointCloudT::Ptr> show_cloud;
		show_cloud.push_back(temp);
		show_cloud.push_back(cloud2);
		show_cloud.push_back(final);
		visualizeSlices(show_cloud);
*/	}
/*	writeFile("saveScore.txt", saveScore);
	vector<PointCloudT::Ptr> show_cloud;
	show_cloud.push_back(cloud1);
	show_cloud.push_back(cloud2);
	visualizeSlices(show_cloud);
*/	return mini;
}

float FeatureExtraction::getAlignScore(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2, float degree)
{
	int times = 360.0/degree;
	PointCloudT::Ptr temp(new PointCloudT);
	*temp = *cloud1;
	float mini = 65535;
	vector<float> saveScore;
	// Eigen::Matrix4f transformation;
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
		grid.setLeafSize (leaf, leaf, leaf);
/*		grid.setInputCloud (object);
		grid.filter (*object);
		grid.setInputCloud (scene);
		grid.filter (*scene);

		PointCloudNT::Ptr temp_object (new PointCloudT);

		for (int i=0; i<object->points.size(); i++)
		{
		 if (object->points[i].x < 0.0)
		  {
			 temp_object->push_back(object->points[i]);
			 object->erase(object->points.begin() + i);
			 i--;
		  }
		}
		pcl::PCDWriter writer;
		// writer.write<PointNT> ("zzz.pcd", *temp_object, false);
		pcl::io::loadPCDFile<PointNT> ("zzz.pcd", *temp_object);
		// *object = *temp_object;
		for (int i=0; i<2; i++)
		{
		 *object += *temp_object;
		}

*/		// Estimate normals for scene
	//	pcl::console::print_highlight ("Estimating scene normals...\n");
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
		}

	/*	if (align.hasConverged ())
		{
			pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
			float value = (float) align.getInliers ().size ()/scene->size ();
			pcl::console::print_info ("FitnessScore/MyScore: %0.9f/%f\n", score, value);
		}
		else
		{
			pcl::console::print_error ("Alignment failed!\n");
		}

		if (i == 9)
		{
			pcl::io::savePCDFile("slice1.pcd", *temp);
			pcl::io::savePCDFile("slice2.pcd", *cloud2);
		}

		vector<PointCloudNT::Ptr> show_cloud;
		show_cloud.push_back(object);
		show_cloud.push_back(object_aligned);
		show_cloud.push_back(scene);
		visualizeSlices(show_cloud);
*/	}
	/*writeFile("saveScore.txt", saveScore);
	vector<PointCloudT::Ptr> show_cloud;
	show_cloud.push_back(cloud1);
	show_cloud.push_back(cloud2);
	visualizeSlices(show_cloud);*/
	return mini;
}

void FeatureExtraction::selectSlices()
{
	int end_index = 3;
	float d = 5.0;

	vector<float> zero_order;
	vector<float> lefts;
	vector<float> rights;

	printf("[INFO] The selected slice number is %d \n", (int)this->slices.size()-end_index-1);

	for (int i = 1; i < this->slices.size()-end_index; i++)
	{
		this->extracted_slices.push_back(this->slices[i]);
		this->slices_span.push_back(pair<float, float>((this->slices[i]->points[0].z - this->epsilon), (this->slices[i]->points[0].z + this->epsilon)));

		PointCloudT::Ptr left_slice = this->slices[i-1];
		PointCloudT::Ptr right_slice = this->slices[i+1];

		float left = getAlignScore(this->slices[i], left_slice, d);
		float right = getAlignScore(this->slices[i], right_slice, d);

		zero_order.push_back(max(abs(left), abs(right)));

		lefts.push_back(left);
		rights.push_back(right);

        printf("[INFO] slice index: %d, left: %.8f, right: %.8f \n", i, left, right);
		if (this->order == 0)
		{
			if ((max(abs(left), abs(right)) > this->threshold) || (this->slices[i]->points.size() < 2*this->slice_points))
			{
				this->trimmed_slice_index.push_back(i-1);
			}
		}
	}

	writeFile("0_order.txt", zero_order);
	writeFile("left.txt", lefts);
	writeFile("right.txt", rights);

    printf("______________________________________________\n");
    printf("|             Slices Spans INFO              |\n");
	for (int i = 0; i < this->slices_span.size(); i++)
	{
		printf("#%d : start: %f  end: %f \n", i, this->slices_span[i].first, this->slices_span[i].second);
    }
    printf("______________________________________________\n");
	visualizeSlices(this->extracted_slices);

	printf("______________________________________________\n");
    printf("|            Trimmed slice index             |\n");
	for(int i = 0 ; i < this->trimmed_slice_index.size(); i++)
	{
		printf("%d\n", this->trimmed_slice_index[i]);
    }
    printf("______________________________________________\n");

    vector<PointCloudT::Ptr> reserved_slices;
    for (int i = 0; i < this->extracted_slices.size(); i++)
    {
    	bool flag = true;
    	for(int j = 0 ; j < this->trimmed_slice_index.size(); j++)
    	{
    		if (this->trimmed_slice_index[j] == i)
    			flag = false;
    	}
    	if (flag == true)
    		reserved_slices.push_back(this->extracted_slices[i]);
    }
    visualizeSlices(reserved_slices);
}

/*void FeatureExtraction::selectSlices()
{
	int end_index = 3;
	this->extracted_slices.push_back(this->slices[0]);
	this->slices_span.push_back(pair<float, float>(this->base, this->slices[0]->points[0].z));
	float start = this->slices[0]->points[0].z;
	float d = 90;//5.0;
	
	vector<float> zero_order;
	vector<float> first_order;
	vector<float> lefts;
	vector<float> rights;

	zero_order.push_back(0);
	for (int i = 1; i < this->slices.size()-end_index; i++)
	{
		PointCloudT::Ptr left_slice = this->slices[i-1];
		PointCloudT::Ptr right_slice = this->slices[i+1];

	//	float left = getICPScore(this->slices[i], left_slice, d);
	//	float right = getICPScore(this->slices[i], right_slice, d);

		float left = getAlignScore(this->slices[i], left_slice, d);
		float right = getAlignScore(this->slices[i], right_slice, d);

		zero_order.push_back(max(abs(left), abs(right)));

		lefts.push_back(left);
		rights.push_back(right);

        printf("[INFO] slice index: %d, left: %.8f, right: %.8f \n", i, left, right);
		if (this->order == 0)
		{
			if ( max(abs(left), abs(right)) > this->threshold && this->slices[i]->points.size() > this->slice_points )
			{
				this->extracted_slices.push_back(this->slices[i]);
				this->slices_span.push_back(pair<float, float>(start, this->slices[i]->points[0].z));
				start = this->slices[i]->points[0].z;
			}
		}
	}
	zero_order.push_back(0);
	first_order.push_back(0);
	if (this->order == 1)
	{
		for (int i = 1; i < zero_order.size(); i++)
		{
			float left_d = zero_order[i] - zero_order[i-1];
			float right_d = zero_order[i+1] - zero_order[i];
			first_order.push_back(left_d);
			if (abs(left_d) > this->threshold || abs(right_d) > this->threshold)
			{
				this->extracted_slices.push_back(this->slices[i]);
				this->slices_span.push_back(pair<float, float>(start, this->slices[i]->points[0].z));
				start = this->slices[i]->points[0].z;
			}
		}
	}


	writeFile("0_order.txt", zero_order);
	writeFile("1_order.txt" ,first_order);
	writeFile("left.txt", lefts);
	writeFile("right.txt", rights);


	this->extracted_slices.push_back(this->slices[this->slices.size()-end_index]);
	this->slices_span.push_back(pair<float, float>(start, this->slices[this->slices.size()-1]->points[0].z));
	 visualizeSlices(this->extracted_slices);
	
	// sort(this->extracted_slices.begin(), this->extracted_slices.end(), sortSlice);
	vector<PointCloudT::Ptr> trim_slices;
	vector<pair<float, float> > trim_spans;

	vector<pair<int, int> > spans_index;
	int p1 = 0;
	bool isclose = false;
	int rate = 2;
	if (this->slices_span[0].second - this->slices_span[0].first <= (this->interval * rate + this->epsilon))
		isclose = true;
	for (int i = 1; i < this->slices_span.size(); i++)
	{
		if (this->slices_span[i].second - this->slices_span[i].first <= this->interval * rate + this->epsilon)
		{
			if (isclose == false)
			{
				i--;
				spans_index.push_back(pair<float, float>(p1, i));
				p1 = i + 1;
				isclose = true;
			}
		}
		else
		{
			spans_index.push_back(pair<float, float>(p1, i-1));
			p1 = i;
			isclose = false;
		}
	}
	spans_index.push_back(pair<float, float>(p1, this->slices_span.size()-1));

	for (int i = 0; i < spans_index.size(); i ++ )
	{
		PointCloudT::Ptr cloud_temp(new PointCloudT);
		*cloud_temp = *this->extracted_slices[(int)((spans_index[i].second + spans_index[i].first)*0.5)];
		trim_slices.push_back(cloud_temp);
		trim_spans.push_back(pair<float, float>(this->slices_span[spans_index[i].first].first, this->slices_span[spans_index[i].second].second));
	}

    printf("______________________________________________\n");
    printf("|             Slices Spans INFO              |\n");
	for (int i = 0; i < this->slices_span.size(); i++)
	{
		printf("#%d : start: %f  end: %f \n", i, this->slices_span[i].first, this->slices_span[i].second);
    }
    printf("______________________________________________\n");
	visualizeSlices(this->extracted_slices);
	for(int i = 0; i < trim_slices.size(); i++)
	{
		for(int j = 0; j < trim_slices[i]->points.size(); j++)
		{
			trim_slices[i]->points[j].z = (trim_spans[i].first + trim_spans[i].second)*1.0 / 2;
		}
	}
	this->extracted_slices = trim_slices;
	this->slices_span = trim_spans;
	visualizeSlices(this->extracted_slices);
    printf("______________________________________________\n");
    printf("|          Trimed Slices Spans INFO          |\n");
	for (int i = 0; i < this->slices_span.size(); i++)
	{
		printf("#%d : start: %f  end: %f \n", i, this->slices_span[i].first, this->slices_span[i].second);
    }
    printf("______________________________________________\n");
    printf("|                 Trimed index               |\n");
	for(int i = 0 ; i < spans_index.size(); i++)
	{
        printf("%d ~ %d \n", spans_index[i].first, spans_index[i].second);
    }
    printf("______________________________________________\n");
}*/

// calculate entropy, no longer using in this version
float FeatureExtraction::KL(vector<float> p, vector<float> q)
{
	float entropy = 0.0;
	if (p.size() != q.size())
		return 0.0;
	for (int i = 0; i < p.size(); i++)
	{
		if (p[i]*q[i] != 0.0)
			entropy += p[i]*log(p[i]/q[i]);
	}
	return entropy;
}

// seperate points into 3 dimensional map
map<string, vector<float> > FeatureExtraction::seperateXY(vector<vector<float> > data)
{
    map<string, vector<float> > res;
	for (int i = 0; i < data.size(); i++)
	{
		res["x"].push_back(data[i][0]);
		res["y"].push_back(data[i][1]);
		res["z"].push_back(data[i][2]);
	}
	return res;
}

// using x axis to seperate data into reserved set and left set
map<string, vector<float> > FeatureExtraction::seperateData(vector<vector<float> > data)
{
    map<string, vector<float> > res;
	for (int i = 0; i < data.size(); i++)
	{
		if (data[i][0] <= 0)
		{
			res["remain_x"].push_back(data[i][0]);
			res["remain_y"].push_back(data[i][1]);
		}
		else
		{
			res["left_x"].push_back(data[i][0]);
			res["left_y"].push_back(data[i][1]);
		}
    }
	return res;
}
float FeatureExtraction::min(vector<float> data)
{
	if (data.size() == 0)
		return -1;
	float res = data[0];
	for (int i = 0; i < data.size(); i++)
		if (data[i] < res)
			res = data[i];
	return res;
}
float FeatureExtraction::max(vector<float> data)
{
	if (data.size() == 0)
		return -1;
	float res = data[0];
	for (int i = 0; i < data.size(); i++)
		if (data[i] > res)
			res = data[i];
	return res;
}
float FeatureExtraction::min(float data1, float data2)
{
	if (data1 < data2)
		return data1;
	return data2;
}
float FeatureExtraction::max(float data1, float data2)
{
	if (data1 > data2)
		return data1;
	return data2;
}
void FeatureExtraction::minusMean(vector<float> &data)
{
	float sum = 0.0;
	for (int i = 0; i < data.size(); i++)
		sum += data[i];
	float average = sum / data.size();
	for (int i = 0; i < data.size(); i++)
		data[i] -= average;
}

// calculate entropy between two distribution, no longer using in this version
float FeatureExtraction::getEntropy(vector<float> data1, vector<float> data2, int bins)
{
//	minusMean(data1);
//	minusMean(data2);
	float min_1 = min(data1);
    float min_2 = min(data2);
	float max_1 = max(data1);
	float max_2 = max(data2);
	float mini = max(min_1, min_2);
	float maxi = min(max_1, max_2);
	float dist = (maxi-mini) / bins;
	map<int, int> map1;
	map<int, int> map2;
	vector<float> prob1;
    vector<float> prob2;
	for (int i = 0; i < bins; i++)
	{
		map1[i] = 0;
		map2[i] = 0;
	}
	for (int i = 0; i < data1.size(); i++)
		if (data1[i] > mini && data1[i] < maxi)
			map1[int((data1[i]-mini)/dist)] ++;
	for (int i = 0; i < data2.size(); i++)
		if (data2[i] > mini && data2[i] < maxi)
			map2[int((data2[i]-mini)/dist)] ++;
	map<int, int>::iterator it;
	for (it = map1.begin(); it != map1.end(); ++it)
		prob1.push_back(it->second*1.0/data1.size());
	for (it = map2.begin(); it != map2.end(); ++it)
		prob2.push_back(it->second*1.0/data2.size());
	float entropy = KL(prob1, prob2);
	return entropy;
}
void FeatureExtraction::writeFile(string filename, vector<float> data)
{
    ofstream file;
    file.open(filename.c_str());
    for (int i = 0; i < data.size(); i++)
    {
        file << data[i] <<"\n";
    }
    file.close();
}
void FeatureExtraction::writeFile(string filename, vector<int> data)
{
    ofstream file;
    file.open(filename.c_str());
    for (int i = 0; i < data.size(); i++)
    {
        file << data[i] <<"\n";
    }
    file.close();
}
void FeatureExtraction::writeFile(string filename, vector<pair<float, float> > data)
{
    ofstream file;
    file.open(filename.c_str());
    for (int i = 0; i < data.size(); i++)
    {
        file << data[i].first << " " << data[i].second <<"\n";
    }
    file.close();
}

// Using KL divergence to select points
PointCloudT::Ptr FeatureExtraction::getKeyPoint(PointCloudT::Ptr cloud, int bins)
{
    vector<vector<float> > res;
	int iteration = cloud->points.size();
    for (int i = 0; i < cloud->points.size(); i++)
    {
        vector<float> temp;
        temp.push_back(cloud->points[i].x);
        temp.push_back(cloud->points[i].y);
        temp.push_back(cloud->points[i].z);
        res.push_back(temp);
    }

	vector<float> save_x;
	vector<float> save_y;
	for (int i = 0; i < res.size(); i ++)
	{
		save_x.push_back(res[i][0]);
		save_y.push_back(res[i][1]);
	}
	// printf("save x size: %d \n", save_x.size());
	// printf("save y size: %d \n", save_y.size());
    map<string, vector<float> > seperated_data = seperateData(res);
	vector<float> entropys_x;
    vector<float> entropys_y;
	int remain_index = seperated_data["remain_x"].size();
	// entropys_x.push_back(getEntropy(seperated_data["remain_x"], seperated_data["left_x"], bins));
	// entropys_y.push_back(getEntropy(seperated_data["remain_y"], seperated_data["left_y"], bins));
	// entropys_x.push_back(getEntropy(save_x, seperated_data["remain_x"], bins));
	// entropys_y.push_back(getEntropy(save_y, seperated_data["remain_y"], bins));
	// entropys_x.push_back(getEntropy(seperated_data["remain_x"], save_x, bins));
	// entropys_y.push_back(getEntropy(seperated_data["remain_y"], save_y, bins));
	entropys_x.push_back(getKLDivergence(save_x, seperated_data["remain_x"], bins));
	entropys_y.push_back(getKLDivergence(save_y, seperated_data["remain_y"], bins));

	int number = 0;
	int nearest_max = 30;
	int nearest = 0;

	PointCloudT::Ptr keypoints(new PointCloudT);
	for (int i = 0; i < iteration; i++)
	{
		vector<float> remain_x(seperated_data["remain_x"]);
		vector<float> remain_y(seperated_data["remain_y"]);
		vector<float> left_x(seperated_data["left_x"]);
        vector<float> left_y(seperated_data["left_y"]);
		int size_start = entropys_x.size();
		for (int j = 0; j < left_x.size(); j++)
		{
			float x_value = left_x[j];
			float y_value = left_y[j];
			remain_x.push_back(x_value);
			remain_y.push_back(y_value);
			left_x.erase(left_x.begin()+j);
            left_y.erase(left_y.begin()+j);
			// float entropy_x = getEntropy(remain_x, left_x, bins);
            // float entropy_y = getEntropy(remain_y, left_y, bins);
			// float entropy_x = getEntropy(save_x, remain_x, bins);
            // float entropy_y = getEntropy(save_y, remain_y, bins);
			// float entropy_x = getEntropy(remain_x, save_x, bins);
            // float entropy_y = getEntropy(remain_y, save_y, bins);
			float entropy_x = getKLDivergence(save_x, remain_x, bins);
			float entropy_y = getKLDivergence(save_y, remain_y, bins);

			if (entropy_x < entropys_x[entropys_x.size()-1] && entropy_y < entropys_y[entropys_y.size()-1])
            {
				entropys_x.push_back(entropy_x);
				entropys_y.push_back(entropy_y);
				seperated_data["remain_x"].push_back(x_value);
				seperated_data["remain_y"].push_back(y_value);
				seperated_data["left_x"].erase(seperated_data["left_x"].begin()+j);
                seperated_data["left_y"].erase(seperated_data["left_y"].begin()+j);
				j--;
				// if (nearest < nearest_max)
				{
					PointT point(x_value, y_value, cloud->points[0].z);
					keypoints->push_back(point);
					nearest ++;
				}
			}
		}
		int size_end = entropys_x.size();
		if (size_start == size_end)
			number++;
		if (number > 10)
			break;
	}
	
	// int size1 = keypoints->points.size();
	// for (int i = 0; i < keypoints->points.size(); i++)
	// {
	// 	for (int j = i + 1; j < keypoints->points.size(); j++)
	// 	{
	// 		float dist = (keypoints->points[i].x - keypoints->points[j].x)*
	// 					(keypoints->points[i].x - keypoints->points[j].x)+
	// 					(keypoints->points[i].y - keypoints->points[j].y)*
	// 					(keypoints->points[i].y - keypoints->points[j].y)+
	// 					(keypoints->points[i].z - keypoints->points[j].z)*
	// 					(keypoints->points[i].z - keypoints->points[j].z);
	// 		dist = sqrt(dist);
	// 		if (dist <= 0.001)
	// 		{
	// 			keypoints->points.erase(keypoints->points.begin()+j);
	// 			break;
	// 		}
	// 	}
	// }
	// int size2 = keypoints->points.size();
	// printf("size 1 is: %d, size 2 is : %d \n", size1, size2);
	writeFile("entropy_x.txt", entropys_x);
	writeFile("entropy_y.txt", entropys_y);
	writeFile("remain_x.txt",seperated_data["remain_x"]);
	writeFile("remain_y.txt",seperated_data["remain_y"]);
	// cout << entropys_x.size() << endl;
	// cout << entropys_y.size() << endl;
	// printf("remain start index: %d \n", remain_index);

	vector<int> remain_indexs;
	remain_indexs.push_back(remain_index);
	writeFile("remain_index.txt", remain_indexs);
	writeFile("all_x.txt", save_x);
	writeFile("all_y.txt", save_y);
	return keypoints;
}

void FeatureExtraction::rotatePointCloud(PointCloudT::Ptr &cloud, float degree, char mode)
{
	Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
	float d = degree / 180 * M_PI;
    if (mode == 'z')
    {
        matrix(0,0) = cos(d);
        matrix(0,1) = -sin(d);
        matrix(1,0) = sin(d);
        matrix(1,1) = cos(d);
    }
    else if (mode == 'y')
    {
        matrix(0,0) = cos(d);
        matrix(0,2) = -sin(d);
        matrix(2,0) = sin(d);
        matrix(2,2) = cos(d);
    }
    else if (mode == 'x')
    {
        matrix(1,1) = cos(d);
        matrix(1,2) = -sin(d);
        matrix(2,1) = sin(d);
        matrix(2,2) = cos(d);
    }
    else
    {
        cout << "[ERROR] Insert true mode index !" << endl;
    }
	pcl::transformPointCloud(*cloud, *cloud, matrix);
}

// rotate 180 degrees twice to extract all key points
PointCloudT::Ptr FeatureExtraction::getAllKeyPoint(PointCloudT::Ptr cloud, int bins)
{
	PointCloudT::Ptr cloudin (new PointCloudT);
	*cloudin = *cloud;
	PointCloudT::Ptr res(new PointCloudT);
	// RGB rgbs[] = {
    //         RGB(255,   0,   0),
    //         RGB(255, 165,   0),
    //         RGB(255, 255,   0),
    //         RGB(  0, 255,   0),
    //         RGB(  0, 127, 255),
    //         RGB(  0,   0, 255),
    //         RGB(139,   0, 255),
    //         RGB(  0,   0,   0),
    //         RGB(255, 255,   0),
	// 		RGB(255, 127,   0),
	// 		RGB(255, 127, 127),
	// 		RGB(255, 127, 255)};
	for (int i = 0; i < 2; i ++ )
	{
		float degree = 180.0;
		rotatePointCloud(cloudin, degree, 'z');
		// printf("degree: %f \n", degree*(i+1));
		PointCloudT::Ptr temp = getKeyPoint(cloudin, bins);
		rotatePointCloud(temp, -1.0*degree*(i+1), 'z');
		*res += *temp;
	}

	int size1 = res->points.size();
	// for (int i = 0; i < res->points.size(); i++)
	// {
	// 	for (int j = i + 1; j < res->points.size(); j++)
	// 	{
	// 		float dist = (res->points[i].x - res->points[j].x)*
	// 					(res->points[i].x - res->points[j].x)+
	// 					(res->points[i].y - res->points[j].y)*
	// 					(res->points[i].y - res->points[j].y)+
	// 					(res->points[i].z - res->points[j].z)*
	// 					(res->points[i].z - res->points[j].z);
	// 		dist = sqrt(dist);
	// 		if (dist <= 0.001)
	// 		{
	// 			res->points.erase(res->points.begin()+j);
	// 			res->width --;
	// 			// printf("****size1: %d", res->width);
	// 			// printf("****size2: %d", res->points.size());
	// 			break;
	// 		}
	// 	}
	// }
	int size2 = res->points.size();
	// printf("*size1: %d", res->width);
	// printf("*size2: %d", res->points.size());
	// res->width = res->points.size();
    printf("[INFO] size 1 is: %d, size 2 is : %d \n", size1, size2);
	// for (int i = 0; )
	return res;
}

void FeatureExtraction::extractKeyPoints()
{
    printf("[INFO] Earsing Similar Points ... \n");
	for (int i = 0; i < this->extracted_slices.size(); i++)
	{
		PointCloudT::Ptr key = getAllKeyPoint(this->extracted_slices[i], 25);
		this->key_points.push_back(key);
	}
}

void FeatureExtraction::visualizePointCloud ( PointCloudT::Ptr cloud, RGB rgb )
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
void FeatureExtraction::visualizePointCloud ( PointCloudT::Ptr* cloud, RGB* rgb, int number)
{
	pcl::visualization::PCLVisualizer viewer("cloud");
    for (int i = 0; i < number; i++)
    {
        ColorHandlerT handle (cloud[i], rgb[i].getR(), rgb[i].getG(), rgb[i].getB());   
        stringstream ss;
        string str;
        ss << i;
        str = "cloud_"+ss.str();
        viewer.addPointCloud (cloud[i], handle, str);
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, str);
    }
    viewer.addCoordinateSystem (0.1, 0, 0, 0);
	viewer.setBackgroundColor(0., 0., 0., 0); 
	// viewer.initCameraParameters ();
  	while (!viewer.wasStopped ())
  	{
  		viewer.spinOnce ();
  	}
}

// KL divergence KL(Q||P)
float FeatureExtraction::getKLDivergence(vector<float> data, vector<float> predict, int bins)
{
	float mini = min(data);
	float maxi = max(data);
	float dist = (maxi - mini) / bins;
	vector<float> spans;
	vector<float> P;
	vector<float> Q;
	vector<float> empty;
	float res = 0.0;
	for (int i = 0; i <= bins; i ++)
	{
		spans.push_back(mini + i * dist);
	}
	map<int, vector<float> > p_count;
	map<int, vector<float> > q_count;
	for (int i = 0; i < data.size(); i++)
	{
		int index = (int) ((data[i] - mini) / dist);
		if (index != bins)
		{
			p_count[index].push_back(data[i]);
			q_count[index] = empty;
		}
		else
		{
			p_count[index-1].push_back(data[i]);
			q_count[index-1] = empty;
		}
	}

	map<pair<float, float>, float> p_map;
	for (int i = 0; i < spans.size() - 1; i ++)
	{
		pair<float, float> temp(spans[i], spans[i+1]);
		p_map[temp] = p_count[i].size()*1.0 / data.size();
	}
	map<pair<float, float>, float>::iterator it;
	for (it = p_map.begin(); it != p_map.end(); ++it)
	{
		P.push_back(it->second);
	}

	map<pair<float, float>, float> q_map;
	for (int i = 0; i < predict.size(); i++)
	{
		if (predict[i] >= mini && predict[i] <= maxi)
		{
			int index = (int) ((predict[i] - mini) / dist);
			if (index != bins)
			{
				q_count[index].push_back(predict[i]);
			}
			else
			{
				q_count[index-1].push_back(predict[i]);
			}
		}
	}
	for (int i = 0; i < spans.size() - 1; i ++)
	{
		pair<float, float> temp(spans[i], spans[i+1]);
		q_map[temp] = q_count[i].size()*1.0 / predict.size();
	}
	float all_prob = 0.0;
	for (it = q_map.begin(); it != q_map.end(); ++it)
	{
		all_prob += it->second;
	}
	for (it = q_map.begin(); it != q_map.end(); ++it)
	{
		Q.push_back(it->second / all_prob);
	}

/*	for (int i = 0; i < P.size(); i ++)
		printf("%f ", P[i]);
	printf("\n");
	printf("-------------------------------\n");
	for (int i = 0; i < Q.size(); i ++)
		printf("%f ", Q[i]);
	printf("\n");*/

	for (int i = 0; i < P.size(); i ++)
	{
		if (P[i] != 0.0 && Q[i] != 0.0)
		{
			res += Q[i]*log(Q[i]*1.0/P[i]);
		}
	}

	return res;
}
