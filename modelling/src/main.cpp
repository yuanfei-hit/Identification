#include "head.h"
#include "modelling.h"
// #include "pose_estimate.h"
#include "extract_feature.h"
#include "model.h"
#include "model_builder.h"
#include "recog.h"


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
bool getCommandBool(char* command)
{
	bool result;
	stringstream ss;
	ss << string(command);
	ss >> result;
	return result;
}
string getCommandString(char* command)
{
	return string(command);
}
PointCloudT::Ptr loadPointCloud ( string filename )
{
	PointCloudT::Ptr cloud( new PointCloudT );
	pcl::PCDReader reader;
	reader.read<PointT> ( filename, *cloud );
	return cloud;
}
// ----------------------------------------------------------------------
//              main function
// ----------------------------------------------------------------------
int main(int argc, char** argv)
{
    if (argc < 2)
    {
        printf("____________________________________________________\n");
        printf("                     Usage:                         \n");
        printf(" rosrun modelling main_node [--modelling]|[--feature]|[--build]|[--read]|[--recog] [option] \n");
        printf(" --modelling: modelling module                      \n");
        printf(" --feature: feature extraction module               \n");
        printf(" --build: comparison database construction module   \n");
        printf(" --read: database reading module                    \n");
        printf(" --recog: recognition module                        \n");
        printf("____________________________________________________\n");
        return (-1);
    }
    string cmd = getCommandString(argv[1]);

    if (cmd == "--modelling")
    {
        // string config = "/home/alan/modelling/config/properties.yaml";
        if (argc != 5)
        {
            cout << "***************     Usage:     ***************" << endl;
            cout << "rosrun modelling main_node --modelling [id] [object name] [config path]" << endl;
            return (-1);
        }
        int id = getCommandInt(argv[2]);
        string object_name = getCommandString(argv[3]);
        string config = getCommandString(argv[4]);
        printf("[INFO] Modelling process start. \n");
        printf("______________________________________________\n");
        printf("| ID:           %d\n", id);
        printf("| Object name:  %s\n", object_name.c_str());
        printf("| Config:       %s\n", config.c_str());
        printf("|____________________________________________|\n");
        string date = ""+object_name+".pcd";
        Modelling demo_model(id, config, date);
        PointCloudT::Ptr cloud = demo_model.getModel();
    }
    else if (cmd == "--feature")
    {
        if (argc != 5)
        {
            cout << "***************     Usage:     ***************" << endl;
            cout << "rosrun modelling main_node --feature [id] [original point cloud] [config path]" << endl;
            return (-1);
        }
        int id = getCommandInt(argv[2]);
        string name = getCommandString(argv[3]);
        string properties_filename = getCommandString(argv[4]);
        printf("[INFO] Feature extraction process start. \n");
        FeatureExtraction feature(id, name, properties_filename);
    }
    else if (cmd == "--build")
    {
        cout << "***************     Usage:     ***************" << endl;
        cout << "rosrun modelling main_node --build [*.model path]" << endl;
        vector<string> filenames;
        for (int i = 2; i < argc; i++)
        {
            printf("[INFO] Load models: %s \n", argv[i]);
            filenames.push_back(string(argv[i]));
        }
        printf("[INFO] Building process start. \n");
        ModelBuilder builder = ModelBuilder(filenames);
    }
    else if (cmd == "--read")
    {        
        if (argc != 3)
        {
            cout << "***************     Usage:     ***************" << endl;
            cout << "rosrun modelling main_node --read [id]" << endl;
            return (-1);
        }
        printf("[INFO] Reading database. \n");
        Database database;
        database.readDatabase("database");
        // database.info();
        stringstream ss;
        int id;
        ss << argv[2] ;
        ss >> id;
        printf("[INFO] Searching information for a ID \n");
        int index = database.search(id);

		printf("____________________________________\n");
		printf("[ model_id ]: %d ", database.model_ids[index]);
		printf("\n");

		printf("[ similar_surface_model_ids   ]: ");
		for (int i = 0; i < database.similar_surface_model_ids[index].size(); i++)
			printf("%d ", database.similar_surface_model_ids[index][i]);
		printf("\n");

		printf("[ similar_slice_model_ids     ]: ");
		for (int i = 0; i < database.similar_slice_model_ids[index].size(); i++)
			printf("%d ", database.similar_slice_model_ids[index][i]);
		printf("\n");

		printf("[ similar_stiffness_model_ids ]: ");
		for (int i = 0; i < database.similar_stiffness_model_ids[index].size(); i++)
			printf("%d ", database.similar_stiffness_model_ids[index][i]);
		printf("\n");

		for (int i = 0; i < database.slice_ids[index].size(); i++)
		{
			printf("[ optimal_slice_ids ]: %d ---> [ slice_heights ]: %f ---> [ dist_similar_surface_model_ids ]: ", database.slice_ids[index][i], database.slice_heights[index][i]);
			for (int j = 0; j < database.dist_similar_surface_model_ids[index][i].size(); j++)
				printf("%d ", database.dist_similar_surface_model_ids[index][i][j]);
			printf("\n");
		}
		printf("|__________________________________| \n");
    }
    else if (cmd == "--recog")
    {
    	if (argc != 4)
		{
			cout << "***************     Usage:     ***************" << endl;
			cout << "rosrun modelling main_node --recog [id] [common file name]" << endl;
			return (-1);
		}

    	string database_filename = ros::package::getPath("knowledge_base") + "/models/database";

		int id = getCommandInt(argv[2]);
		string common_file_name = getCommandString(argv[3]);
		Recog recog = Recog(database_filename);
		PointCloudT  visual_cloud;
		PointCloudNT tactile_0_cloud;
		PointCloudNT tactile_1_cloud;
		PointCloudNT tactile_2_cloud;
		std::string path = ros::package::getPath("identifier") + "/ExpData/" + common_file_name;
		pcl::PCDReader reader;
		reader.read<PointT>  (path + "target.pcd", visual_cloud);
		reader.read<PointNT> (path + "finger_0_NT.pcd", tactile_0_cloud);
		reader.read<PointNT> (path + "finger_1_NT.pcd", tactile_1_cloud);
		reader.read<PointNT> (path + "finger_2_NT.pcd", tactile_2_cloud);
		std::vector<float> stiffness;
		string filename = path + "contact_feature.txt";
		fstream in(filename.c_str());
		string line;
		while(getline(in,line))
		{
			if (line.empty())
				break;
			else
			{
				stringstream ss(line);
				float value;
				ss >> value;
				stiffness.push_back(value);
			}
		}
		std::vector<PointCloudT::Ptr> show_cloud;
		PointCloudT::Ptr temp1 (new PointCloudT);
		PointCloudT::Ptr temp2 (new PointCloudT);
		PointCloudT::Ptr temp3 (new PointCloudT);
		PointCloudT::Ptr temp4 (new PointCloudT);
		PointCloudT::Ptr temp5 (new PointCloudT);
	    copyPointCloud (visual_cloud, *temp1);
		show_cloud.push_back(temp1);
		bool flag = recog.isRecognitionSuccess(id, visual_cloud, tactile_0_cloud, tactile_1_cloud, tactile_2_cloud, stiffness);
		std::cout << "Recognition Flag: " << flag << std::endl;
		copyPointCloud (visual_cloud, *temp2);
		copyPointCloud (tactile_0_cloud, *temp3);
		copyPointCloud (tactile_1_cloud, *temp4);
		copyPointCloud (tactile_2_cloud, *temp5);
		show_cloud.push_back(temp2);
		show_cloud.push_back(temp3);
		show_cloud.push_back(temp4);
		show_cloud.push_back(temp5);
		recog.visualizePointCloud(show_cloud);
    }
    else
    {
        printf("____________________________________________________\n");
        printf("                     Usage:                         \n");
        printf(" rosrun modelling main_node [--modelling]|[--feature]|[--build]|[--read]|[--recog] [option] \n");
        printf(" --modelling: modelling module                      \n");
        printf(" --feature: feature extraction module               \n");
        printf(" --build: comparison database construction module   \n");
        printf(" --read: database reading module                    \n");
        // printf(" --recog: recognition module                        \n");
        printf("____________________________________________________\n");
        return (-1);
    }
    return (0);
}

