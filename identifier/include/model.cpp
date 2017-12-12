#include "model.h"

Model::Model()
{
    this->slice_num = 0;
    this->raw_num = 0;
    this->id = -1;
    this->name = "null";
    this->height = -1;
    this->texture.push_back(1000); //mean of stiffness
    this->texture.push_back(100);  //Standard Deviation of stiffness
    // info();
}
Model::Model(string filename)
{
    readModel(filename);
//    printf("slice_size: %d\n", this->raw.size());
    // visualizeSlices(this->raw);
    // visualizeSlices(this->slices);
    // visualizeSlices(this->keys);
}
Model::Model(int id, string name, vector<PointCloudT::Ptr> raw, vector<PointCloudT::Ptr> slices, vector<PointCloudT::Ptr> keys, vector<pair<float, float> > spans, vector<int> trimmed_slice_index, float height)
{
    this->raw = raw;
    this->slices = slices;
    this->keys = keys;
    this->spans = spans;
    this->trimmed_slice_index = trimmed_slice_index;
    this->slice_num = this->slices.size();
    this->raw_num = this->raw.size();
    this->texture.push_back(1000); //mean of stiffness
    this->texture.push_back(100);  //deviation of stiffness
    this->id = id;
    this->name = name;
    float max = -65535;
    for (int i = 0; i < this->spans.size(); i++)
        if (this->spans[i].second > max)
            max = this->spans[i].second;
    this->height = height;
    visualizeSlices(this->raw);
    visualizeSlices(this->slices);
//    visualizeSlices(this->keys);

    // info();
}
Model::~Model()
{
    
}

void Model::readModel(string filename)
{
    printf("[INFO] Reading Model from File %s ... \n", filename.c_str());
    fstream in(filename.c_str());
    string line;
    while(getline(in,line))
    {
        stringstream ss(line);
        vector<string> values;
        if (line[0] == '#')
            continue;
        string value;
        while(ss >> value)
        {
            values.push_back(value);
        }
        if (values.size() == 0)
            continue;
        if (values[0] == "RAWDATA")
            break;
        if (values[0] == "ID")
        {
            stringstream ss_temp;
            ss_temp << values[1];
            ss_temp >> this->id;
        } 
        else if (values[0] == "NAME")
        {
            this->name = values[1];
        }
        else if (values[0] == "SLICE_NUM")
        {
            stringstream ss_temp;
            ss_temp << values[1];
            ss_temp >> this->slice_num;
        }
        else if (values[0] == "RAW_NUM")
        {
            stringstream ss_temp;
            ss_temp << values[1];
            ss_temp >> this->raw_num;
        }
        else if (values[0] == "TEXTURE")
        {
            for (int i = 1; i < values.size(); i++)
			{
				stringstream ss_temp;
				float float_temp;
				ss_temp << values[i];
				ss_temp >> float_temp;
				this->texture.push_back(float_temp);
			}
        }
        else if (values[0] == "HEIGHT")
        {
            stringstream ss_temp;
            ss_temp << values[1];
            ss_temp >> this->height;
        }
        else if (values[0] == "SPAN")
        {
            for (int i = 1; i < values.size()-1;)
            {
                stringstream ss1,ss2;
                float v1,v2;
                ss1 << values[i];
                ss2 << values[i+1];
                ss1 >> v1;
                ss2 >> v2;
                pair<float, float> span(v1, v2);
                this->spans.push_back(span);
                i += 2;
                // printf("#%d: %f, %f \n", i, v1, v2);
            }
        }
		else if (values[0] == "TRIMMED_SLICE_INDEX")
		{
			for (int i = 1; i < values.size(); i++)
			{
				stringstream ss_temp;
				int int_temp;
				ss_temp << values[i];
				ss_temp >> int_temp;
				this->trimmed_slice_index.push_back(int_temp);
			}
		}
    }

    bool isRAW = false;
    bool isPointCloud = false;
    bool isKey = false;
    PointCloudT::Ptr cloud (new PointCloudT);
    while(getline(in,line))
    {
        if (line == "POINTSDATA")
            break;
        if (line.find("RAW") != -1)
        {
            cloud->points.clear();
            cloud->width = 0;
            getline(in, line);
            isRAW = true;
        }
        if (line.find("END") != -1)
        {
            PointCloudT::Ptr slice_temp(new PointCloudT);
            *slice_temp = *cloud;
            this->raw.push_back(slice_temp);
            isRAW =false;
        }
        if (isRAW)
        {
            stringstream ss(line);
            float temp[3];
            for (int i = 0; i < 3; i++)
                ss >> temp[i];
            PointT point(temp[0],temp[1],temp[2]);
            cloud->push_back(point);
        }
    }

    cloud->points.clear();
    cloud->width = 0;
    while(getline(in,line))
    {
        if (line == "POINTSDATAK")
            break;
        if (line.find("SLICE") != -1)
        {
            cloud->points.clear();
            cloud->width = 0;
            getline(in, line);
            isPointCloud = true;
        }
        if (line.find("END") != -1)
        {
            PointCloudT::Ptr slice_temp(new PointCloudT);
            *slice_temp = *cloud;
            this->slices.push_back(slice_temp);
            isPointCloud =false;
        }
        if (isPointCloud)
        {
            stringstream ss(line);
            float temp[3];
            for (int i = 0; i < 3; i++)
                ss >> temp[i];
            PointT point(temp[0],temp[1],temp[2]);
            cloud->push_back(point);
        }
    }

    // info();
    cloud->points.clear();
    cloud->width = 0;
    while(getline(in,line))
    {
        if (line == "FILEEND")
            break;
        if (line.find("KEY") != -1)
        {
            cloud->points.clear();
            cloud->width = 0;
            getline(in, line);
            isKey = true;
        }
        if (line.find("END") != -1)
        {
            PointCloudT::Ptr slice_temp(new PointCloudT);
            *slice_temp = *cloud;
            this->keys.push_back(slice_temp);
            isKey =false;
        }
        if (isKey)
        {
            stringstream ss(line);
            float temp[3];
            for (int i = 0; i < 3; i++)
                ss >> temp[i];
            PointT point(temp[0],temp[1],temp[2]);
            cloud->push_back(point);
        }
    }
    // printf("visualize starting ...\n");
    // visualizeSlices(this->raw);
    // visualizeSlices(this->slices);
    // visualizeSlices(this->keys);
    // printf("visualize ending ...\n");
    info();
}
void Model::saveModel(string filename)
{
    // visualizePointCloud(this->slices);
    printf("[ERROR] Saving Model into File %s ... \n", filename.c_str());
    ofstream file;
    file.open(filename.c_str());
    file << "#Point Cloud Model Data Format \n";
    file << "ID " << this->id << "\n";
    file << "NAME " << this->name << "\n";
    file << "SLICE_NUM " << this->slice_num << "\n";
    file << "RAW_NUM " << this->raw_num << "\n";
    file << "TEXTURE ";
    for (int i = 0; i < this->texture.size(); i ++)
    	file << this->texture[i] << " ";
    file << "\n";
    file << "HEIGHT " << this->height << "\n";
    file << "SPAN ";
    for (int i = 0; i < this->spans.size(); i ++)
        file << this->spans[i].first << " " << this->spans[i].second << " ";
    file << "\n";
    file << "TRIMMED_SLICE_INDEX ";
    for (int i = 0; i < this->trimmed_slice_index.size(); i ++)
    	file << this->trimmed_slice_index[i] << " ";
    file << "\n";
    file << "RAWDATA\n";
    for (int i = 0; i < this->raw.size(); i++)
    {
        file << "RAW" << i << "\n";
        for (int j = 0; j < this->raw[i]->points.size(); j++)
        {
            file << this->raw[i]->points[j].x << " "
                    << this->raw[i]->points[j].y << " "
                        << this->raw[i]->points[j].z << " \n";
        }
        file << "END" << i << "\n";
    }

    file << "POINTSDATA\n";
    for (int i = 0; i < this->slices.size(); i ++)
    {
        file << "SLICE" << i << "\n";
        for (int j = 0; j < this->slices[i]->points.size(); j ++)
        {
            file << this->slices[i]->points[j].x << " "
                    << this->slices[i]->points[j].y << " "
                        << this->slices[i]->points[j].z << " \n";
        }
        file << "END" << i << "\n";
    }
    file << "POINTSDATAK\n";
    for (int i = 0; i < this->keys.size(); i ++)
    {
        file << "KEY" << i << "\n";
        for (int j = 0; j < this->keys[i]->points.size(); j ++)
        {
            file << this->keys[i]->points[j].x << " "
                    << this->keys[i]->points[j].y << " "
                        << this->keys[i]->points[j].z << " \n";
        }
        file << "END" << i << "\n";
    }
    file << "FILEEND\n";
    info();
}

void Model::info()
{
    printf("______________________________________________\n");
    printf("|                   MODEL INFO               |\n");
    printf("| id:               %d\n", this->id);
    printf("| name:             %s\n", this->name.c_str());
    printf("| slice_num:        %d\n", this->slice_num);
    printf("| raw_num:          %d\n", this->raw_num);
    printf("| texture             \n");
    printf("| ---mean:          %f\n", this->texture[0]);
    printf("| ---std:           %f\n", this->texture[1]);
    printf("| height:           %f\n", this->height);
    printf("| key num:          %d\n", (int)this->keys.size());
    printf("| span num:         %d\n", (int)this->spans.size());
    printf("| trimmed_slice num:%d\n", (int)this->trimmed_slice_index.size());
    printf("|____________________________________________|\n");
}

void Model::visualizeSlices(vector<PointCloudT::Ptr> visual)
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

Database::Database()
{
    this->database_size = 0;
}
Database::Database(string filename)
{
    readDatabase(filename);
}
Database::~Database()
{

}
void Database::insert(int model_id,
	    			  vector<int> similar_surface_model_ids,
					  vector<int> similar_slice_model_ids,
					  vector<int> similar_stiffness_model_ids,
					  vector<int> slice_ids,
					  vector<float> slice_heights,
					  vector<vector<int> > dist_similar_surface_model_ids)
{
    this->model_ids.push_back(model_id);
    this->similar_surface_model_ids.push_back(similar_surface_model_ids);
    this->similar_slice_model_ids.push_back(similar_slice_model_ids);
    this->similar_stiffness_model_ids.push_back(similar_stiffness_model_ids);
    this->slice_ids.push_back(slice_ids);
    this->slice_heights.push_back(slice_heights);
    this->dist_similar_surface_model_ids.push_back(dist_similar_surface_model_ids);

    this->database_size ++;
}
void Database::saveDatabase(string filename)
{
    ofstream file;
    file.open(filename.c_str());
    file << "#Model Database File Format\n";
    for (int i = 0; i < this->database_size; i ++)
    {
        file << "TABLE" << i << "\n";

        file << "model_id " << this->model_ids[i] << "\n";

        file << "similar_surface_model_ids ";
        for (int j = 0; j < this->similar_surface_model_ids[i].size(); j++)
            file << this->similar_surface_model_ids[i][j] << " ";
        file << "\n";

        file << "similar_slice_model_ids ";
		for (int j = 0; j < this->similar_slice_model_ids[i].size(); j++)
			file << this->similar_slice_model_ids[i][j] << " ";
		file << "\n";

		file << "similar_stiffness_model_ids ";
		for (int j = 0; j < this->similar_stiffness_model_ids[i].size(); j++)
			file << this->similar_stiffness_model_ids[i][j] << " ";
		file << "\n";

        file << "slice_ids ";
        for (int j = 0; j < this->slice_ids[i].size(); j++)
        	file << this->slice_ids[i][j] << " ";
        file << "\n";

        file << "slice_heights ";
        for (int j = 0; j < this->slice_heights[i].size(); j++)
            file << this->slice_heights[i][j] << " ";
        file << "\n";

        file << "dist_similar_surface_model_ids " << this->dist_similar_surface_model_ids[i].size() <<"\n";
        for (int j = 0; j < this->dist_similar_surface_model_ids[i].size(); j++)
        {
        	for (int k = 0; k < this->dist_similar_surface_model_ids[i][j].size(); k++)
        		file << this->dist_similar_surface_model_ids[i][j][k] << " ";
        	file << "\n";
        }

        file << "END" << i << "\n";
    }
    file << "FILEEND\n";
}


void Database::readDatabase(string filename)
{
    fstream file;
    file.open(filename.c_str());
    string line;
    bool flag = false;
    int model_id;
	vector<int> similar_sur_model_ids;
	vector<int> similar_sli_model_ids;
	vector<int> similar_sti_model_ids;
	vector<int> slice_ids;
	vector<float> slice_heights;
	vector<vector<int> > dist_similar_sur_model_ids;
    while(getline(file, line))
    {
        if (line.find("#") != -1)
            continue;
        if (line.find("END") != -1)
        {
            this->insert(model_id,
            		     similar_sur_model_ids,
						 similar_sli_model_ids,
						 similar_sti_model_ids,
						 slice_ids,
						 slice_heights,
						 dist_similar_sur_model_ids);
            model_id = -1;
            similar_sur_model_ids.clear();
            similar_sli_model_ids.clear();
            similar_sti_model_ids.clear();
            slice_ids.clear();
            slice_heights.clear();
            dist_similar_sur_model_ids.clear();
            getline(file, line);
        }
        if (line.find("TABLE") != -1)
            continue;
        stringstream ss(line);
        string value;
        vector<string> values;
        while(ss >> value)
        {
            values.push_back(value);;
        }
        if (values[0] == "model_id")
        {
            stringstream ss1;
            ss1 << values[1];
            ss1 >> model_id;
        }
        else if (values[0] == "similar_surface_model_ids")
        {
            for (int i=1; i<values.size(); i++)
            {
            	stringstream ss1;
            	int temp;
            	ss1 << values[i];
            	ss1 >> temp;
            	similar_sur_model_ids.push_back(temp);
            }
        }
        else if (values[0] == "similar_slice_model_ids")
        {
        	for (int i=1; i<values.size(); i++)
			{
				stringstream ss1;
				int temp;
				ss1 << values[i];
				ss1 >> temp;
				similar_sli_model_ids.push_back(temp);
			}
        }
        else if (values[0] == "similar_stiffness_model_ids")
		{
        	for (int i=1; i<values.size(); i++)
			{
				stringstream ss1;
				int temp;
				ss1 << values[i];
				ss1 >> temp;
				similar_sti_model_ids.push_back(temp);
			}
		}
        else if (values[0] == "slice_ids")
		{
        	for (int i=1; i<values.size(); i++)
        	{
        		stringstream ss1;
        		int temp;
        		ss1 << values[i];
        		ss1 >> temp;
        		slice_ids.push_back(temp);
			}
		}
        else if (values[0] == "slice_heights")
		{
        	for (int i=1; i<values.size(); i++)
        	{
        		stringstream ss1;
        		float temp;
        		ss1 << values[i];
        		ss1 >> temp;
        		slice_heights.push_back(temp);
        	}
		}
        else if (values[0] == "dist_similar_surface_model_ids")
        {
            stringstream ss1;
            int size;
            ss1 << values[1];
            ss1 >> size;
            for (int i = 0; i < size; i ++)
            {
                getline(file, line);
                stringstream ss2(line);
                vector<int> temp;
                int value_temp;
                while(ss2 >> value_temp)
                {
                    temp.push_back(value_temp);
                }
                dist_similar_sur_model_ids.push_back(temp);
            }
        }
    }
}
int Database::size()
{
    return this->database_size;
}
void Database::info()
{
    for (int i = 0; i < this->database_size; i++)
    {
        printf("____________________________________\n");
        printf("| id: %d ", this->model_ids[i]);
        printf("\n");

        printf("| similar_surface_model_ids:   ");
        for (int j = 0; j < this->similar_surface_model_ids[i].size(); j++)
        	printf("%d ", this->similar_surface_model_ids[i][j]);
        printf("\n");

        printf("| similar_slice_model_ids:     ");
        for (int j = 0; j < this->similar_slice_model_ids[i].size(); j++)
        	printf("%d ", this->similar_slice_model_ids[i][j]);
        printf("\n");

        printf("| similar_stiffness_model_ids: ");
        for (int j = 0; j < this->similar_stiffness_model_ids[i].size(); j++)
        	printf("%d ", this->similar_stiffness_model_ids[i][j]);
        printf("\n");

        for (int j = 0; j < this->slice_ids[i].size(); j++)
        {
            printf("| optimal_slice_ids: %d ---> | slice_heights: %f ---> | dist_similar_surface_model_ids: ", this->slice_ids[i][j], this->slice_heights[i][j]);
            for (int k = 0; k < this->dist_similar_surface_model_ids[i][j].size(); k++)
                printf("%d ", this->dist_similar_surface_model_ids[i][j][k]);
            printf("\n");
        }
    }
    printf("|__________________________________| \n");
}

int Database::search(int id)
{
    int index = -1;
    for (int i = 0; i < this->model_ids.size(); i++)
    {
        if (this->model_ids[i] == id)
        {
            index =  i;
            break;
        }
    }
    return index;
}
