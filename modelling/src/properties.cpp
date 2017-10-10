#include "properties.h"

SamplingProperties::SamplingProperties()
{
    this->search_radius = 0.0;
    this->polynomial_fit = true;
    this->polynomial_order = 0;
    this->radius_search = 0.005;
    this->depth = 5;
}
SamplingProperties::SamplingProperties(string filename, int index)
{
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    stringstream ss;
    string index_str;
    ss << index;
    ss >> index_str;
    FileNode node = fs["sampling"+index_str];
    this->search_radius = (float)node["search_radius"];
    int temp = (int)node["polynomial_fit"];
    if (temp == 1)
        this->polynomial_fit = true;
    else
        this->polynomial_fit = false;
    this->polynomial_order = (int)node["polynomial_order"];
    this->radius_search = (float)node["radius_search"];
    this->depth = (int)node["depth"];
}
void SamplingProperties::info()
{
    printf("______________________________________________\n");
    printf("|          SAMPLING PROPERTIES INFO          |\n");
    printf("| polynomial_fit:   %d                       |\n", this->polynomial_fit);
    printf("| polynomial_order: %d                       |\n", this->polynomial_order);
    printf("| radius_search:    %f                       |\n", this->radius_search);
    printf("| depth:            %d                       |\n", this->depth);
    printf("______________________________________________\n");
}
SamplingProperties::~SamplingProperties()
{

}
VoxelGridProperties::VoxelGridProperties()
{
    this->size = 0;
}
VoxelGridProperties::VoxelGridProperties(string filename, int index)
{
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    stringstream ss;
    string index_str;
    ss << index;
    ss >> index_str;
    FileNode node = fs["voxel"+index_str];
    this->size = (float)node["size"];
}
void VoxelGridProperties::info()
{
    
    printf("______________________________________________\n");
    printf("|         VOXEL GRID PROPERTIES INFO         |\n");
    printf("| size:             %f                       |\n", this->size);
    printf("______________________________________________\n");
}
VoxelGridProperties::~VoxelGridProperties()
{

}

SliceProperties::SliceProperties()
{
    this->slices_size = 9;
    this->interval = 0.01;
    this->epsilon = 0.0001;
    this->search_base = 0;
    this->search_num = 100;
    this->threshold = 5e-7;
    this->slice_points = 100;
    this->valid_height = 0.03;
    this->order = 0;
}
SliceProperties::SliceProperties(string filename)
{
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    FileNode node = fs["slices"];
    this->slices_size = (int)node["slices_size"];
    this->interval = (float)node["interval"];
    this->epsilon = (float)node["epsilon"];
    this->search_base = (float)node["search_base"];
    this->search_num = (int)node["search_num"];
    this->threshold = (float)node["threshold"];
    this->slice_points = (int)node["slice_points"];
    this->valid_height = (float)node["valid_height"];
    this->order = (int)node["order"];
}
void SliceProperties::info()
{
    printf("______________________________________________\n");
    printf("|           SLIDES PROPERTIES INFO           |\n");
    printf("| slices_size:      %d\n", this->slices_size);
    printf("| interval:         %f\n", this->interval);
    printf("| epsilon:          %f\n", this->epsilon);
    printf("| search_base:      %f\n", this->search_base);
    printf("| search_num:       %d\n", this->search_num);
    printf("| threshold:        %f\n", this->threshold);
    printf("| slice_points:     %d\n", this->slice_points);
    printf("| valid_height:     %f\n", this->valid_height);
    printf("| order:            %d\n", this->order);
    printf("|____________________________________________|\n");
}
SliceProperties::~SliceProperties()
{

}


ModellingProperties::ModellingProperties()
{
    this->x_bias = 0.;
    this->y_bias = 0.;
    this->z_bias = 0.;
    this->x_degree = 0.;
    this->y_degree = 0.;
    this->z_degree = 0.;
    this->clouds_filename.push_back("f1.pcd");
    this->clouds_filename.push_back("f2.pcd");
    this->clouds_filename.push_back("f3.pcd");
    this->clouds_filename.push_back("f4.pcd");
    this->matrix_filename = "transform_matrix.txt";
    // this->center_filename = "object_center.txt";
    this->rotate_degree = 0.;
    this->pose_degree = 0.0;
}
ModellingProperties::ModellingProperties(string filename)
{
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    FileNode node = fs["modelling"];
    this->x_bias = (float)node["x_bias"];
    this->y_bias = (float)node["y_bias"];
    this->z_bias = (float)node["z_bias"];
    this->x_degree = (float)node["x_degree"];
    this->y_degree = (float)node["y_degree"];
    this->z_degree = (float)node["z_degree"];
    string filename_temp = (string)node["clouds_filename"];
    // cout << filename_temp << endl;
    istringstream iss(filename_temp);
    do
    {
        string sub;
        iss >> sub;
        // printf("name: %s \n", sub.c_str());
        if (sub.find(".pcd")!=-1)
            this->clouds_filename.push_back(sub);
    }while(iss);
    this->matrix_filename = (string)node["matrix_filename"];
    // this->center_filename = (string)node["center_filename"];
    this->rotate_degree = (float)node["rotate_degree"];
    this->pose_degree = (float)node["pose_degree"];
}
void ModellingProperties::info()
{
    printf("______________________________________________\n");
    printf("|          MODELLING PROPERTIES INFO         |\n");
    printf("| x_bias:           %f\n", this->x_bias);
    printf("| y_bias:           %f\n", this->y_bias);
    printf("| z_bias:           %f\n", this->z_bias);
    printf("| x_degree:         %f\n", this->x_degree);
    printf("| y_degree:         %f\n", this->y_degree);
    printf("| z_degre:          %f\n", this->z_degree);
    for (int i = 0; i < this->clouds_filename.size(); i++)
    {
    printf("| clouds_filename%d:%s\n", i, this->clouds_filename[i].c_str());
    }
    printf("| matrix_filename:  %s\n", this->matrix_filename.c_str());
    printf("| rotate_degree:    %f\n", this->rotate_degree);
    printf("|____________________________________________|\n");
}
ModellingProperties::~ModellingProperties()
{
    
}

OutlierProperties::OutlierProperties()
{
    this->meank = 50;
    this->threshold = 0.01;
}
OutlierProperties::OutlierProperties(string filename)
{
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    FileNode node = fs["outlier"];
    this->meank = (int)node["meank"];
    this->threshold = (float)node["threshold"];
}
void OutlierProperties::info()
{
    printf("______________________________________________\n");
    printf("|       REMOVE OUTLIER PROPERTIES INFO        \n");
    printf("| meank:            %d\n", this->meank);
    printf("| threshold:        %f\n", this->threshold);
    printf("|____________________________________________|\n");
}
OutlierProperties::~OutlierProperties()
{

}

PoissonProperties::PoissonProperties()
{    
    this->search_radius = 0.025;
    this->polynomial_fit = true;
    this->polynomial_order = 5;
    this->ksearch = 20;
    this->confidence = false; 
    this->degree = 2; 
    this->depth = 8; 
    this->ISO = 8; 
    this->manifold = false; 
    this->polygon = false; 
    this->smooth = 3.0; 
    this->scale = 1.25;
    this->solver = 8;
    this->min_depth = 7;
}
PoissonProperties::PoissonProperties(string filename)
{
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    FileNode node = fs["poisson"];
    this->search_radius = (float)node["search_radius"];
    this->polynomial_fit = (int)node["polynomial_fit"];
    this->polynomial_order = (int)node["polynomial_order"];
    this->ksearch = (int)node["ksearch"];
    this->confidence = (int)node["confidence"]; 
    this->degree = (int)node["degree"]; 
    this->depth = (int)node["depth"]; 
    this->ISO = (int)node["ISO"]; 
    this->manifold = (int)node["manifold"]; 
    this->polygon = (int)node["polygon"]; 
    this->smooth = (float)node["smooth"]; 
    this->scale = (float)node["scale"];
    this->solver = (int)node["solver"];
    this->min_depth = (int)node["min_depth"];
}
void PoissonProperties::info()
{
    printf("______________________________________________\n");
    printf("|      POISSON SAMPLING PROPERTIES INFO      |\n");
    printf("| search_radius:    %f\n", this->search_radius);
    printf("| polynomial_fit:   %d\n", this->polynomial_fit);
    printf("| polynomial_order: %d\n", this->polynomial_order);
    printf("| ksearch:          %d\n", this->ksearch);
    printf("| confidence:       %d\n", this->confidence);
    printf("| degree:           %d\n", this->degree);
    printf("| depth:            %d\n", this->depth);
    printf("| ISO:              %d\n", this->ISO);
    printf("| manifold:         %d\n", this->manifold);
    printf("| polygon:          %d\n", this->polygon);
    printf("| smooth:           %f\n", this->smooth);
    printf("| scale:            %f\n", this->scale);
    printf("| solver:           %d\n", this->solver);;
    printf("| min_depth:        %d\n", this->min_depth);
    printf("|____________________________________________|\n");
}
PoissonProperties::~PoissonProperties()
{

}
