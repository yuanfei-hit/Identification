#include "head.h"
#include "properties.h"
#include "extract_feature.h"
#include "model.h"


PointCloudT::Ptr loadPointCloud (string filename )
{
	PointCloudT::Ptr cloud( new PointCloudT );
	pcl::PCDReader reader;
	reader.read<PointT> ( filename, *cloud );
	return cloud;
}

PointCloudNT::Ptr loadPointCloudNT (string filename )
{
	PointCloudNT::Ptr cloud( new PointCloudNT );
	pcl::PCDReader reader;
	reader.read<PointNT> ( filename, *cloud );
	return cloud;
}

int getCommandInt(char* command)
{
	int result;
	stringstream ss;
	ss << string(command);
	ss >> result;
	return result;
}
float getCommandFloat(char* command)
{
	float result;
	stringstream ss;
	ss << string(command);
	ss >> result;
	return result;
}
bool getCommandBool(string command)
{
	bool result;
	stringstream ss;
	ss << string(command);
	ss >> result;
	return result;
}

Eigen::Matrix4f readMatrix(string filename)
{    
    fstream in(filename.c_str());
    string line;
    Eigen::Matrix4f result;
    int index = 0;
    while(getline(in,line))
    {
        stringstream ss(line);
        float value;
        while(ss>>value)
        {
            result(index/4, index%4) = value;
            index++;
        }
    }
    return result;
}

float getICPScore(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2)
{
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	Eigen::Matrix4f matrix;
	icp.setInputSource(cloud1);
	icp.setInputTarget(cloud2);
	PointCloudT align;
	PointCloudT::Ptr final(new PointCloudT);
	icp.align(align);
	*final = align;
	float score = icp.getFitnessScore();
	Eigen::Matrix4f transformation = icp.getFinalTransformation();
	return score;
}

void visualizePointCloud(vector<PointCloudT::Ptr> clouds)
{
   RGB rgbs[] = {
            RGB(255,   0,   0),
            RGB(255, 165,   0),
            RGB(255, 255,   0),
            RGB(  0, 255,   0),
            RGB(  0, 127, 255),
            RGB(  0,   0, 255),
            RGB(139,   0, 255),
            RGB(  0,   0,   0),
            RGB(255, 255,   0),
			RGB(255, 127,   0),
			RGB(255, 127, 127),
			RGB(255, 127, 255)};
	pcl::visualization::PCLVisualizer viewer("cloud");
	for (int i = 0; i < clouds.size(); i++)
	{
		ColorHandlerT handle (clouds[i], rgbs[i%12].getR(), rgbs[i%12].getG(), rgbs[i%12].getB());
		stringstream ss;
		string index;
		ss << i;
		ss >> index;
		viewer.addPointCloud (clouds[i], handle, "cloud_visualization"+index);
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_visualization"+index);
	}
	viewer.addCoordinateSystem (0.1, 0, 0, 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); 
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
}

void visualizePointCloud(vector<PointCloudNT::Ptr> clouds)
{
   RGB rgbs[] = {
            RGB(255,   0,   0),
            RGB(255, 165,   0),
            RGB(255, 255,   0),
            RGB(  0, 255,   0),
            RGB(  0, 127, 255),
            RGB(  0,   0, 255),
            RGB(139,   0, 255),
            RGB(  0,   0,   0),
            RGB(255, 255,   0),
			RGB(255, 127,   0),
			RGB(255, 127, 127),
			RGB(255, 127, 255)};
	pcl::visualization::PCLVisualizer viewer("cloud");
	for (int i = 0; i < clouds.size(); i++)
	{
		ColorHandlerNT handle (clouds[i], rgbs[i%12].getR(), rgbs[i%12].getG(), rgbs[i%12].getB());
		stringstream ss;
		string index;
		ss << i;
		ss >> index;
		viewer.addPointCloud (clouds[i], handle, "cloud_visualization"+index);
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_visualization"+index);
	}
	viewer.addCoordinateSystem (0.1, 0, 0, 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
}

void writeFile(string filename, vector<float> data)
{
    ofstream file;
    file.open(filename.c_str());
    for (int i = 0; i < data.size(); i++)
    {
        file << data[i] <<"\n";
    }
    file.close();
}

void saveMatrix(string filename, Eigen::MatrixXf matrix)
{
	ofstream file;
    file.open(filename.c_str());
	int num = matrix.rows();
	cout << matrix.rows() << endl;
	cout << matrix.cols() << endl;
    for (int i = 0; i < num; i++)
    {
        for (int j = 0; j < num; j++)
		{
            file << matrix(i, j) << " ";
        }
        file << "\n";
        }
    file.close();
}

void calculate(PointCloudT::Ptr cloud1, PointCloudT::Ptr cloud2, float &score, float &rate)
{
    PointCloudNT::Ptr object_source_cloud (new PointCloudNT);
    PointCloudNT::Ptr scene_target_cloud (new PointCloudNT);

    copyPointCloud (*cloud1, *object_source_cloud);
    copyPointCloud (*cloud2, *scene_target_cloud);

    PointCloudNT::Ptr object (new PointCloudNT);
    PointCloudNT::Ptr object_aligned (new PointCloudNT);
    PointCloudNT::Ptr scene (new PointCloudNT);
    FeatureCloudT::Ptr object_features (new FeatureCloudT);
    FeatureCloudT::Ptr scene_features (new FeatureCloudT);

    *object = *object_source_cloud;
    *scene  = *scene_target_cloud;

    const float leaf = 0.005f;

    pcl::NormalEstimationOMP<PointNT,PointNT> nest;
    nest.setRadiusSearch (0.02);
    nest.setInputCloud (object);
    nest.compute (*object);
    nest.setInputCloud (scene);
    nest.compute (*scene);

    FeatureEstimationT fest;
    fest.setRadiusSearch (0.04);
    fest.setInputCloud (object);
    fest.setInputNormals (object);
    fest.compute (*object_features);
    fest.setInputCloud (scene);
    fest.setInputNormals (scene);
    fest.compute (*scene_features);

    pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
    align.setInputSource (object);
    align.setSourceFeatures (object_features);
    align.setInputTarget (scene);
    align.setTargetFeatures (scene_features);
    align.setMaximumIterations (50000); // Number of RANSAC iterations
    align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness (5); // Number of nearest features to use
    align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance (4.5f * leaf); // Inlier threshold
    align.setInlierFraction (0.01f); // Required inlier fraction for accepting a pose hypothesis
    {
        align.align (*object_aligned);
    }

    float fitness_score = (float) align.getFitnessScore();
	Eigen::Matrix4f transformation;
	score = fitness_score;


    // if (align.hasConverged ())
    // {
        transformation = align.getFinalTransformation ();
    	// pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
	
	// }
    // else
    // {
    //     pcl::console::print_error ("Alignment failed!\n");
    // }
	rate = align.getInliers().size()*1.0 / object->size();
	// printf("score: %f, rate: %f\n", score, rate);
	// PointCloudT::Ptr show(new PointCloudT);
	// pcl::transformPointCloud(*cloud1, *show, transformation);
	// vector<PointCloudT::Ptr> clouds;
	// clouds.push_back(cloud1);
	// clouds.push_back(cloud2);
	// clouds.push_back(show);
	// visualizePointCloud(clouds);
	// return fitness_score;
}






int main(int argc, char** argv)
{
	if (string(argv[1]) == "--show")
	{
		vector<PointCloudT::Ptr> clouds;
		for (int i = 2; i < argc; i++)
		{
			PointCloudT::Ptr cloud = loadPointCloud(string(argv[i]));
			clouds.push_back(cloud);
		}
		visualizePointCloud(clouds);
	}
	else if (string(argv[1]) == "--showNT")
	{
		vector<PointCloudNT::Ptr> clouds;
		for (int i = 2; i < argc; i++)
		{
			PointCloudNT::Ptr cloud = loadPointCloudNT(string(argv[i]));
			clouds.push_back(cloud);
		}
		visualizePointCloud(clouds);
	}
	else if (string(argv[1]) == "--showonce")
	{
		Model model1 = Model(string(argv[2]));
		for (int i = 0; i < model1.raw.size(); i++)
		{
			vector<PointCloudT::Ptr> clouds;
			clouds.push_back(model1.raw[i]);
			visualizePointCloud(clouds);
		}
	}
	else if (string(argv[1]) == "--raw")
	{
		Model model = Model(string(argv[2]));
		visualizePointCloud(model.raw);
	}
	else if (string(argv[1]) == "--slice")
	{
		Model model = Model(string(argv[2]));
		visualizePointCloud(model.slices);
	}
	else if (string(argv[1]) == "--keypoint")
	{
		Model model = Model(string(argv[2]));
		visualizePointCloud(model.keys);
	}
	else if (string(argv[1]) == "--span")
	{
		Model model = Model(string(argv[2]));
		for (int i = 0; i < model.spans.size(); i++)
		{
			printf("[ #%d ] %f ~ %f \n", i, model.spans[i].first, model.spans[i].second);
		}
	}
	else if (string(argv[1]) == "--icp")
	{
		Model model1 = Model(string(argv[2]));
		Model model2 = Model(string(argv[3]));
		Eigen::MatrixXf score11(model1.raw.size(), model2.raw.size());
		Eigen::MatrixXf rate11(model1.raw.size(), model2.raw.size());
		Eigen::MatrixXf score22(model1.raw.size(), model2.raw.size());
		Eigen::MatrixXf rate22(model1.raw.size(), model2.raw.size());
		Eigen::MatrixXf max_score(model1.raw.size(), model2.raw.size());
		Eigen::MatrixXf max_rate(model1.raw.size(), model2.raw.size());
		Eigen::MatrixXf min_score(model1.raw.size(), model2.raw.size());
		Eigen::MatrixXf min_rate(model1.raw.size(), model2.raw.size());



		for (int i = 0; i < model1.raw.size(); i++)
		{
			for (int j = 0; j < model2.raw.size(); j++)
			{
				float score1 = 0.0;
				float score2 = 0.0;
				float rate1 = 0.0;
				float rate2 = 0.0;
				calculate(model1.raw[i], model2.raw[j], score1, rate1);
				calculate(model2.raw[j], model1.raw[i], score2, rate2);
				printf("[%d, %d]: %f %f %f %f \n", i, j, score1, rate1, score2, rate2);
				score11(i, j) = score1;
				rate11(i, j) = rate1;
				score22(i, j) = score2;
				rate22(i, j) = rate2;
				printf("[%d, %d]: %f %f %f %f \n", i, j, score11(i, j), rate11(i, j), score22(i, j), rate22(i, j));
				if (score1 < score2)
				{
					min_score(i, j) = score1;
					min_rate(i, j) = rate1;
					max_score(i, j) = score2;
					max_rate(i, j) = rate2;
				}
				else
				{
					min_score(i, j) = score2;
					min_rate(i, j) = rate2;
					max_score(i, j) = score1;
					max_rate(i, j) = rate1;
				}
			}
		}
		saveMatrix("score1.txt", score11);
		saveMatrix("rate1.txt", rate11);
		saveMatrix("score2.txt", score22);
		saveMatrix("rate2.txt", rate22);
		saveMatrix("maxs.txt", max_score);
		saveMatrix("maxr.txt", max_rate);
		saveMatrix("mins.txt", min_score);
		saveMatrix("minr.txt", min_rate);
	}
	else if (string(argv[1]) == "--compare")
	{
		Model model1 = Model(string(argv[2]));
		Model model2 = Model(string(argv[3]));
		int i = getCommandInt(argv[4]);
		int j = getCommandInt(argv[5]);
		vector<PointCloudT::Ptr> clouds;
		clouds.push_back(model1.raw[i]);
		clouds.push_back(model2.raw[j]);
		visualizePointCloud(clouds);
		float score1 = 0.0;
		float score2 = 0.0;
		float rate1 = 0.0;
		float rate2 = 0.0;
		calculate(model1.raw[i], model2.raw[j], score1, rate1);
		calculate(model2.raw[j], model1.raw[i], score2, rate2);
		printf("[%d, %d]: %f %f %f %f \n", i, j, score1, rate1, score2, rate2);

	}
    return (0);
}
