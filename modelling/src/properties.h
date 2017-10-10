#pragma once
#include "head.h"

class SamplingProperties
{
public:
    float search_radius; 
    bool polynomial_fit;
    int polynomial_order; 
    float radius_search;
    int depth;
    SamplingProperties();
    SamplingProperties(string filename, int index);
    void info();
    ~SamplingProperties();
};

class VoxelGridProperties
{
public:
    float size;
    VoxelGridProperties();
    VoxelGridProperties(string filename, int index);
    void info();
    ~VoxelGridProperties();
};

class SliceProperties
{
public:
    int slices_size;
    float interval;
    float epsilon;
    float search_base;
    int search_num;
    float threshold;
    int slice_points;
    float valid_height;
    int order;
    SliceProperties();
    SliceProperties(string filename);
    void info();
    ~SliceProperties();
};

class ModellingProperties
{
public:
    float x_bias;
    float y_bias;
    float z_bias;
    float x_degree;
    float y_degree;
    float z_degree;
    vector<string> clouds_filename;
    string matrix_filename;
    // string center_filename;
    float rotate_degree;
    float pose_degree;
    ModellingProperties();
    ModellingProperties(string filename);
    void info();
    ~ModellingProperties();
};

class OutlierProperties
{
public:
    int meank;
    float threshold;
    OutlierProperties();
    OutlierProperties(string filename);
    void info();
    ~OutlierProperties();
};

class PoissonProperties
{
public:
    float search_radius;
    bool polynomial_fit;
    int polynomial_order;
    int ksearch;
    bool confidence; 
    int degree; 
    int depth; 
    int ISO; 
    bool manifold; 
    bool polygon; 
    float smooth; 
    float scale;
    int solver;
    int min_depth;
    PoissonProperties();
    PoissonProperties(string filename);
    void info();
    ~PoissonProperties();
};
