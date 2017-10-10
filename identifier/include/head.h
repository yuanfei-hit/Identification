#pragma once
#include <ros/ros.h>
#include <signal.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <string.h>
#include <fstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/date_time.hpp>
#include <ros/package.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/conversions.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
#include <typeinfo>
#include <pcl/io/vtk_io.h> 
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/keypoints/harris_3d.h>
#include <math.h>
#include "std_msgs/String.h"

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

typedef pcl::PointXYZ            PointT;
typedef pcl::PointCloud<PointT>  PointCloudT;
typedef pcl::PointNormal         PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef pcl::FPFHSignature33     FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> ColorHandlerT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerNT;

// static float PI = 3.1415926535897932384626;
class RGB
{
public:
	RGB(){this->R=0;this->G=0;this->B=0;};
	RGB(int r, int g, int b){this->R=r;this->G=g;this->B=b;};
	int getR(){return this->R;};
	int getG(){return this->G;};
	int getB(){return this->B;};
private:
	int R;
	int G;
	int B;
};
// class Polar
// {
// public:
// 	float theta;
// 	float rho;
// 	float z;
// 	Polar(float theta, float rho, float z){this->theta = theta; this->rho = rho; this->z = z;};
// };

// no longer use, this class was used to do calibration and pose estimation
class NewPoint
{
public:
    float x;
    float y;
    NewPoint(float x, float y)
    {
        this->x = x;
        this->y = y;
    };
    NewPoint operator+(const NewPoint& point)
    {
        NewPoint result = *this;
        result.x = this->x + point.x;
        result.y = this->y + point.y;
        return result;
    }
};

// this class is for slice nodes
class Slice
{
public:
    int id;
    float height;
    pair<float, float> span;
    Slice(){this->id = -1; this->span.first = -1; this->span.second = -1;this->height = -1;};
    Slice(int id, pair<float, float> span)
    {
        this->id = id; 
        this->span = span;
        this->height = (this->span.first + this->span.second) * 0.50;
        };
    bool operator < (const Slice& slice) const
    {
        if (this->id == slice.id)
            return this->height < slice.height;
        else
            return this->id < slice.id;
    }
};
