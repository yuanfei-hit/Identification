#include "head.h"
#include "modelling.h"
// #include "pose_estimate.h"
#include "extract_feature.h"
#include "model.h"
#include "model_builder.h"

void visualizePointCloud ( PointCloudT::Ptr cloud, RGB rgb )
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
void visualizePointCloud ( PointCloudT::Ptr* cloud, RGB* rgb, int number)
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
        // printf(" --recog: recognition module                        \n");
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

