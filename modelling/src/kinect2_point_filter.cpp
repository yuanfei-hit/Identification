/*
 * identifier.cpp
 *
 *  Created on: 9 May 2017
 *      Author: yuanfei
 */

#include <boost/date_time.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <kdl/frames.hpp>
//#include <identification_msgs/IdentifierControlCommands.h>
//#include <identification_msgs/IdentifierStates.h>
//#include <identification_msgs/TactileProcessorStates.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
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

#include <pcl/registration/icp.h>
#include <string.h>
#include <iostream>
#include <stdio.h>
using namespace std;

typedef pcl::PointXYZ            PointT;
typedef pcl::PointCloud<PointT>  PointCloudT;

typedef pcl::PointNormal         PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef pcl::FPFHSignature33     FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerNT;

PointCloudT::Ptr savepointcloud(new PointCloudT);
int saveindex = 1;
void keyboardEvent(const pcl::visualization::KeyboardEvent &event)
{
    if (event.getKeySym () == "s" && event.keyDown ())
    {
        stringstream ss;
        string str;
        ss << saveindex;
        ss >> str;
	printf("************************************************* \n");
	printf("*                                               * \n");
	printf("*     Saving filtered point cloud to %d.pcd     * \n", saveindex);
	printf("*                                               * \n");
	printf("************************************************* \n");
        pcl::io::savePCDFileASCII(str+".pcd", *savepointcloud);
        saveindex ++;
    }
    else if (event.getKeySym () == "t" && event.keyDown ())
    {
        ros::shutdown();
    }
    else if (event.getKeySym () == "r" && event.keyDown ())
    {
        if (saveindex - 1 >= 1)
	    saveindex --;
        else
            saveindex = 1;
	printf("[     Operating file: %d.pcd     ] \n", saveindex);
    }

}


class IdentifierClass
{
public:
    IdentifierClass();
private:
    std::string  update_crof2marker_transform;
    std::string  used_marker_frame;
    int          num_of_recording_marker_frame;

    PointCloudT  pcl_cloud;
    PointCloudT  pcl_mean_filtered_cloud;
    PointCloudT  pcl_mean_and_passthrough_filtered_cloud;
    PointCloudT  pcl_mean_and_passthrough_and_statistic_filtered_cloud;
    PointCloudT  pcl_mean_and_passthrough_and_statistic_and_radiusoutlier_filtered_cloud;
    PointCloudT  pcl_mean_and_passthrough_and_statistic_and_radiusoutlier_and_downsample_filtered_cloud;

    bool pcl_xxx_filtered_cloud_valid_flag;

    PointCloudT  current_recognition_scene_cloud;



    ros::Subscriber kinect2_point_sub;

    ros::Publisher  kinect2_filtered_point_pub;

    ros::Timer timer0;
    tf::TransformListener listener;
    tf::TransformBroadcaster tf_broadcaster;

    tf::Transform marker2base_transform; //Transform of marker frame to base frame
    tf::Transform cl2base_transform;     //Transform of camera_link frame to base frame

    void timer0EventCallback(const ros::TimerEvent& event);
   // void timer1EventCallback(const ros::TimerEvent& event);
   // void timer2EventCallback(const ros::TimerEvent& event);
   // void tactileProcessorStateCallback(const identification_msgs::TactileProcessorStates& msg);
   // void identifierControlCommandCallback(const identification_msgs::IdentifierControlCommands& msg);
    void kinect2PointFilterCallback(const sensor_msgs::PointCloud2& msg);

    void viewTwoComparePointCloud(const PointCloudT & input_cloud_A,
                                  const PointCloudT & input_cloud_B);
    void viewSegmentPointCloud(const PointCloudT & input_cloud);
    void segmentPointCloud(const PointCloudT::Ptr cloud,
                                 PointCloudT::Ptr plane_cloud,
                                 PointCloudT::Ptr filtered_cloud,
                                 std::vector<pcl::PointIndices> & cluster_indices);
    void recognizeTargetBasedOnVisualPointCloud(const PointCloudT::Ptr source_cloud,
                                                const PointCloudT::Ptr target_cloud,
                                                      Eigen::Matrix4f& transformation,
                                                      float& fitness_score);
    void alignmentPrerejective(const PointCloudNT::Ptr source_cloud,
                               const PointCloudNT::Ptr target_cloud,
                                     Eigen::Matrix4f& transformation,
                                     float& fitness_score);
  //  void setCL2BaseTransform(const std::string&   update_crof2marker_transform,
                //             const std::string&   marker_type,
                  //           const int&           num_of_recording_marker_frame,
                 //            const tf::Transform& marker2base_transform,
                   //                tf::Transform& cl2base_transform);
   // void transformPointCloudFromKinectFrameToWorldFrame(const PointCloudT & source_cloud,
                                                            //  PointCloudT & transformed_cloud);
  //  void recognizeDesiredObjectFromScene(const PointCloudT & scene_cloud,
                                   //      const //identification_msgs::IdentifierControlCommands& identifier_control_commands,
                                   //      const //identification_msgs::TactileProcessorStates&    tactile_processor_states);
};

IdentifierClass::IdentifierClass()
{
    ros::NodeHandle nh("~");

 /*   nh.param("update_crof2marker_transform", update_crof2marker_transform, std::string(""));
    nh.param("used_marker_frame", used_marker_frame, std::string(""));
    nh.param("num_of_recording_marker_frame", num_of_recording_marker_frame, 1);

    pcl_xxx_filtered_cloud_valid_flag = false;

    for (int i=0; i<10; i++)
    {
        fitness_score[i] = 1.0;
        potential_target_fitness_score[i]  = 1.0;

        source_file_index[i] = 0;
        potential_target_source_file_index[i] = 0;
    }

    num_clustered_cloud = 0;
    num_potential_target_clustered_cloud = 0;

    //=== Set constant transform marker frame to base frame ===//
    marker2base_transform.setOrigin(tf::Vector3(0.245, -0.270, 0.02));     //x, y, z
    marker2base_transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));//x, y, z, w

    setCL2BaseTransform(update_crof2marker_transform,
                        used_marker_frame,
                        num_of_recording_marker_frame,
                        marker2base_transform,
                        cl2base_transform);
*/
//	kinect2_point_sub              = nh.subscribe("/kinect2/sd/points", 1, &IdentifierClass::kinect2PointFilterCallback, this);
    kinect2_point_sub              = nh.subscribe("/camera/depth_registered/points", 1, &IdentifierClass::kinect2PointFilterCallback, this);
//    tactile_processor_states_sub   = nh.subscribe("/tactile_processor_states", 1, &IdentifierClass::tactileProcessorStateCallback, this);
//    identifier_control_command_sub = nh.subscribe("/identifier_control_commands", 1, &IdentifierClass::identifierControlCommandCallback, this);

 //   identifier_states_pub       = nh.advertise<identification_msgs::IdentifierStates>("/identifier_states", 1, true);
    kinect2_filtered_point_pub  = nh.advertise<sensor_msgs::PointCloud2>("/kinect2_filtered_points", 1, true);

    timer0 = nh.createTimer(ros::Duration(0.10), &IdentifierClass::timer0EventCallback, this);
  //  timer1 = nh.createTimer(ros::Duration(0.10), &IdentifierClass::timer1EventCallback, this);
  //  timer2 = nh.createTimer(ros::Duration(0.01), &IdentifierClass::timer2EventCallback, this);

//	ROS_WARN_STREAM("zyf_test: " << 1); //for debug
}

void IdentifierClass::timer0EventCallback(const ros::TimerEvent& event)
{
    if (pcl_xxx_filtered_cloud_valid_flag == true)
    {
//		viewTwoComparePointCloud(pcl_mean_filtered_cloud, pcl_mean_and_passthrough_and_statistic_and_radiusoutlier_filtered_cloud);
	    viewSegmentPointCloud(pcl_mean_and_passthrough_and_statistic_and_radiusoutlier_filtered_cloud);

        //=====================================================//
//		std::string path = ros::package::getPath("identifier");
//		pcl::PCDWriter writer;
//		writer.write<PointT> (path + "/PCD/scene.pcd", pcl_mean_and_passthrough_filtered_cloud, false);
        //=====================================================//

        //------------------------------------------------//
        //-------------------for debug--------------------//
/*	    //------------------------------------------------//
        PointCloudT::Ptr source_cloud (new PointCloudT);
        PointCloudT::Ptr target_cloud (new PointCloudT);

        std::string path = ros::package::getPath("identifier");
        pcl::PCDReader reader;
        reader.read<PointT> (path + "/knowledge_database/model_point_cloud/" + "wood" + ".pcd", *source_cloud);
        reader.read<PointT> (path + "/knowledge_database/model_point_cloud/" + "reconstruct_wood" + ".pcd", *target_cloud);

//		reader.read<PointT> (path + "/PCD/tea_box.pcd", *target_cloud);

//		viewTwoComparePointCloud(*source_cloud, *target_cloud);

        //*target_cloud = pcl_mean_and_passthrough_and_statistic_and_radiusoutlier_filtered_cloud;

        Eigen::Matrix4f transformation;
        float fitness_score;
        recognizeTargetBasedOnVisualPointCloud(source_cloud, target_cloud,transformation, fitness_score);
 */	}
}




void IdentifierClass::kinect2PointFilterCallback(const sensor_msgs::PointCloud2& msg)
{
    //camera/depth_registered/points  = 640*480
    //kinect2/sd/points  = 512*424
    //kinect2/qhd/points = 960*540
    //kinect2/hd/points  = 1920*1080
    static int   point_number   = 640*480;
    static int   record_counter = 0;
    static int   record_counter_threshold = 30;//30;
    static int   sum_counter[640*480]     = {0};
    static float sum_value[640*480][3]    = {0};

    fromROSMsg(msg, pcl_cloud);

//	ROS_WARN_STREAM("zyf_test: " << pcl_cloud.points[0]); //for debug

    //sum the valid point data
    for(int i=0; i<point_number; i++)
    {
        if (pcl_cloud.points[i].z > 0)
        {
            sum_value[i][0] += pcl_cloud.points[i].x;
            sum_value[i][1] += pcl_cloud.points[i].y;
            sum_value[i][2] += pcl_cloud.points[i].z;
            sum_counter[i]++;
        }
    }

    record_counter ++;
    if (record_counter == record_counter_threshold)
    {
        // mean filter for a certain organized point clouds//
        record_counter = 0;
        pcl_xxx_filtered_cloud_valid_flag = false;
        PointCloudT  pcl_cloud_temp;
        pcl_cloud_temp = pcl_cloud;
        for(int i=0; i<point_number; i++)
        {
            if (sum_counter[i] != 0)
            {
                pcl_cloud_temp.points[i].x = sum_value[i][0]/sum_counter[i];
                pcl_cloud_temp.points[i].y = sum_value[i][1]/sum_counter[i];
                pcl_cloud_temp.points[i].z = sum_value[i][2]/sum_counter[i];

                sum_value[i][0] = 0.0;
                sum_value[i][1] = 0.0;
                sum_value[i][2] = 0.0;
                sum_counter[i]  = 0;
            }
        }
        pcl_mean_filtered_cloud = pcl_cloud_temp; //organized dataset

        //-------------------------------------------------------------//
        //------------------------------------------------------------//
        //---- Filtering a PointCloud using a PassThrough filter ---- //
        //------------------------------------------------------------//
        // Create the filtering object
        PointCloudT::Ptr cloud (new PointCloudT);
        PointCloudT::Ptr filtered_cloud (new PointCloudT);

        *cloud = pcl_mean_filtered_cloud;

        pcl::PassThrough<PointT> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.5, 1.3);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*filtered_cloud);

        *cloud = *filtered_cloud;

        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("y");
        pass.setFilterLimits (-0.5, 0.25);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*filtered_cloud);

        *cloud = *filtered_cloud;

        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (-0.1, 0.4);
        //pass.setFilterLimitsNegative (true);
        pass.filter (*filtered_cloud);

        pcl_mean_and_passthrough_filtered_cloud = *filtered_cloud;

        //=== Transform point cloud to world coordinate ===//
      //  transformPointCloudFromKinectFrameToWorldFrame(*filtered_cloud, pcl_mean_and_passthrough_filtered_cloud);
    //    pcl_mean_and_passthrough_filtered_cloud.header.frame_id = "base";

        //------------------------------------------------------------//
        // Removing outliers using a StatisticalOutlierRemoval filter //
        //------------------------------------------------------------//
        *cloud = pcl_mean_and_passthrough_filtered_cloud;

        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK (50);
        sor.setStddevMulThresh (0.7);
        sor.filter (*filtered_cloud);

        pcl_mean_and_passthrough_and_statistic_filtered_cloud = *filtered_cloud; //unorganized dataset
    /*	std::cerr << "Cloud before StatisticalOutlierRemoval filtering: " << std::endl;
        std::cerr << *cloud << std::endl;
        std::cerr << "Cloud after StatisticalOutlierRemoval filtering: " << std::endl;
        std::cerr << *filtered_cloud<< std::endl;
    */

        //------------------------------------------------------------//
        //---- Filtering a PointCloud using a RadiusOutlier filter ---//
        //------------------------------------------------------------//
        // Create the filtering object
        *cloud = pcl_mean_and_passthrough_and_statistic_filtered_cloud;
        pcl::RadiusOutlierRemoval<PointT> outrem;
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(0.01);
        outrem.setMinNeighborsInRadius (10);
        outrem.filter (*filtered_cloud);
        pcl_mean_and_passthrough_and_statistic_and_radiusoutlier_filtered_cloud = *filtered_cloud;

        //------------------------------------------------------------//
        //---- Downsampling a PointCloud using a VoxelGrid filter---- //
        //------------------------------------------------------------//
        // Create the filtering object
        pcl::PCLPointCloud2::Ptr pcl_pc2 (new pcl::PCLPointCloud2 ());
        pcl::PCLPointCloud2::Ptr pcl_pc2_filtered (new pcl::PCLPointCloud2 ());

        sensor_msgs::PointCloud2 temp;
        toROSMsg(pcl_mean_and_passthrough_and_statistic_and_radiusoutlier_filtered_cloud, temp);
        pcl_conversions::toPCL(temp, *pcl_pc2);

        pcl::VoxelGrid<pcl::PCLPointCloud2> sorv;
        sorv.setInputCloud (pcl_pc2);
        sorv.setLeafSize (0.01f, 0.01f, 0.01f);
        sorv.filter (*pcl_pc2_filtered);

        pcl_conversions::fromPCL(*pcl_pc2_filtered, temp);
        fromROSMsg(temp, pcl_mean_and_passthrough_and_statistic_and_radiusoutlier_and_downsample_filtered_cloud);
        //------------------------------------------------------------//
        //----------------------filter end----------------------------//
        //------------------------------------------------------------//
        pcl_xxx_filtered_cloud_valid_flag = true;

//		toROSMsg(pcl_mean_filtered_cloud, temp);                                                                //organized dataset
//		toROSMsg(pcl_mean_and_passthrough_filtered_cloud, temp);                                                //unorganized dataset
//		toROSMsg(pcl_mean_and_passthrough_and_statistic_filtered_cloud, temp);                                  //unorganized dataset
        toROSMsg(pcl_mean_and_passthrough_and_statistic_and_radiusoutlier_filtered_cloud, temp);                //unorganized dataset
//		toROSMsg(pcl_mean_and_passthrough_and_statistic_and_radiusoutlier_and_downsample_filtered_cloud, temp); //unorganized dataset

/*		// For reconstruct point cloud model of objects //
        pcl::PointCloud<PointTRGB>  pcl_cloud_rgb_temp;
        fromROSMsg(msg, pcl_cloud_rgb_temp);                         // Get current rgb value from msg.
        copyPointCloud(pcl_mean_filtered_cloud, pcl_cloud_rgb_temp); // combine point data and rgb data.
        toROSMsg(pcl_cloud_rgb_temp, temp);
*/
        kinect2_filtered_point_pub.publish(temp);
    }
}

void IdentifierClass::viewTwoComparePointCloud(const PointCloudT & input_cloud_A,
                                               const PointCloudT & input_cloud_B)
{
    PointCloudT::Ptr cloud_A (new PointCloudT);
    PointCloudT::Ptr cloud_B (new PointCloudT);

    *cloud_A = input_cloud_A;
    *cloud_B = input_cloud_B;

    pcl::visualization::PCLVisualizer viewer ("filtered_point_clouds");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_handler_A (cloud_A, 255, 255, 255);// white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_handler_B (cloud_B, 230,  20,  20);// Red

    viewer.addPointCloud (cloud_A, cloud_color_handler_A, "cloud_A");
    viewer.addPointCloud (cloud_B, cloud_color_handler_B, "cloud_B");

    viewer.addCoordinateSystem (0.2, "cloud_A", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_A");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_B");
    //viewer.setPosition(800, 400); // Setting visualiser window position
    //------------------------------------------------------------------------------------------//
    //----------------------------------Set viewer pose-----------------------------------------//
    //------------------------------------------------------------------------------------------//
    viewer.initCameraParameters ();
    Eigen::Affine3f viewer_pose (Eigen::Affine3f::Identity ());
    viewer_pose = Eigen::Affine3f (Eigen::Translation3f (0,0,-0.5));

    Eigen::Vector3f     pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
    Eigen::Vector3f      up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
    viewer.setCameraPosition(    pos_vector[0],     pos_vector[1],     pos_vector[2],
                             look_at_vector[0], look_at_vector[1], look_at_vector[2],
                                  up_vector[0],      up_vector[1],      up_vector[2]);
    //------------------------------------------------------------------------------------------//
    while (!viewer.wasStopped ()) // Display the visualiser until 'q' key is pressed
    {
        viewer.spinOnce ();
    }
}

void IdentifierClass::viewSegmentPointCloud(const PointCloudT & input_cloud)
{
    PointCloudT::Ptr cloud (new PointCloudT);
    PointCloudT::Ptr plane_cloud (new PointCloudT);
    PointCloudT::Ptr filtered_cloud (new PointCloudT);
    std::vector<pcl::PointIndices> cluster_indices;

    *cloud = input_cloud;
    //------------------------------------------------------------------------------------------//
    //------------------------------segment point cloud-----------------------------------------//
    //------------------------------------------------------------------------------------------//
    ros::WallTime start_time = ros::WallTime::now();

    segmentPointCloud(cloud, plane_cloud, filtered_cloud, cluster_indices);

    double time = ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();
    ROS_INFO_STREAM("Time used for segmenting: " <<time);
    //------------------------------------------------------------------------------------------//
    //------------------------------end of segmentation-----------------------------------------//
    //------------------------------------------------------------------------------------------//
    pcl::visualization::PCLVisualizer viewer ("segment_point_clouds");
    pcl::visualization::PointCloudColorHandlerCustom<PointT> filtered_cloud_color_handler (filtered_cloud, 255, 255, 255);// White
    viewer.addPointCloud (filtered_cloud, filtered_cloud_color_handler, "filtered_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "filtered_cloud");

    pcl::visualization::PointCloudColorHandlerCustom<PointT> plane_cloud_color_handler (plane_cloud, 255, 0, 255);// Magenta
    viewer.addPointCloud (plane_cloud, plane_cloud_color_handler, "plane_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "plane_cloud");

    std::string cloud_actor_map_id;
    int r, g, b;
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        PointCloudT::Ptr cloud_cluster (new PointCloudT);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (filtered_cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::ostringstream oss;
        oss<< j;
        cloud_actor_map_id = "cloud_cluster_" + oss.str();

        ROS_INFO_STREAM("PointCloud representing the Cluster: "<<cloud_actor_map_id <<": "<<cloud_cluster->points.size () << " data points.");

        if (j == 0){
            r = 160; g =  32; b = 240;//Purple
//            std::string path = ros::package::getPath("identifier");
		*savepointcloud = *cloud_cluster;
		pcl::visualization::PCLVisualizer myviewer("cloud");
		pcl::visualization::PointCloudColorHandlerCustom<PointT> handle (savepointcloud, r, g, b);
		myviewer.addPointCloud (savepointcloud, handle, "Viewer");
		myviewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "Viewer");

	    myviewer.addCoordinateSystem (0.1, 0, 0, 0);
		myviewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
	    myviewer.registerKeyboardCallback(keyboardEvent); 
	  	while (!myviewer.wasStopped ())
	  	{
	  	    myviewer.spinOnce ();
	  	}
//            pcl::PCDWriter writer;
//            writer.write<PointT> (path + "/PCD/model.pcd", *cloud_cluster, false);
        }
        if (j == 1){
            r =   0; g =   0; b = 255;//Blue1
        }
        if (j == 2){
            r = 255; g = 165; b =   0;//Orange
        }
        if (j == 3){
            r = 144; g = 238; b = 144;//LightGreen
        }
        if (j == 4){
            r = 230; g =   0; b =   0;//Red
        }

        //pcl::visualization::PointCloudColorHandlerCustom<PointT> segment_cloud_color_handler (cloud_cluster, r, g, b);//
        //viewer.addPointCloud (cloud_cluster, segment_cloud_color_handler, cloud_actor_map_id);
       // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_actor_map_id);

    //	std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        j++;
    }
/*	//------------------------------------------------------------------------------------------//
    //----------------------------------Transform point cloud-----------------------------------//
    //------------------------------------------------------------------------------------------//
    float theta = M_PI/4;
    Eigen::Affine3f transform (Eigen::Affine3f::Identity ());
    transform.translation() << 0.15, 0.1, 0.0;                          // Define a translation on the x, y, z axis.
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));// Theta radians arround Z axis
    std::cout <<"Transform matrix: "<<std::endl<<transform.matrix() << std::endl;

    PointCloudT::Ptr source_cloud (new PointCloudT);
    PointCloudT::Ptr transformed_cloud (new PointCloudT);

    std::string path = ros::package::getPath("identifier");
    pcl::PCDReader reader;
    reader.read<PointT> (path + "/PCD/tea_box.pcd", *source_cloud);

    pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform);

    pcl::visualization::PointCloudColorHandlerCustom<PointT> transformed_cloud_color_handler (transformed_cloud, 139, 0, 0);// DarkRed
    viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
*/	//------------------------------------------------------------------------------------------//
    //------------------------------------------------------------------------------------------//

 /*  viewer.addCoordinateSystem (0.2, "sensor", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    //viewer.setPosition(800, 400); // Setting visualiser window position

    //------------------------------------------------------------------------------------------//
    //----------------------------------Set viewer pose-----------------------------------------//
    //------------------------------------------------------------------------------------------//
    viewer.initCameraParameters ();
    Eigen::Affine3f viewer_pose (Eigen::Affine3f::Identity ());
    viewer_pose = Eigen::Affine3f (Eigen::Translation3f (0,0,-0.5));

    Eigen::Vector3f     pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
    Eigen::Vector3f      up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
    viewer.setCameraPosition(    pos_vector[0],     pos_vector[1],     pos_vector[2],
                             look_at_vector[0], look_at_vector[1], look_at_vector[2],
                                  up_vector[0],      up_vector[1],      up_vector[2]);
    //------------------------------------------------------------------------------------------//
    //------------------------------------------------------------------------------------------//
    while (!viewer.wasStopped ()) // Display the visualiser until 'q' key is pressed
    {
        viewer.spinOnce ();
    }*/
}

void IdentifierClass::segmentPointCloud(const PointCloudT::Ptr cloud,
                                              PointCloudT::Ptr plane_cloud,
                                              PointCloudT::Ptr filtered_cloud,
                                              std::vector<pcl::PointIndices> & cluster_indices)
{
    std::cout << "segmentation input point clouds: " << *cloud << std::endl; //*

    // Create the filtering object: downsample the dataset using a leaf size of 5 mm
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (0.005f, 0.005f, 0.005f);
    vg.filter (*filtered_cloud);
    std::cout << "PointCloud after filtering has: " << filtered_cloud->points.size ()  << " data points." << std::endl; //*

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.008);

    int nr_points = (int) filtered_cloud->points.size ();
    while (filtered_cloud->points.size () > 0.5 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (filtered_cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
          std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud (filtered_cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*plane_cloud);
        std::cout << "PointCloud representing the planar component: " << plane_cloud->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*filtered_cloud);
    }

      // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (filtered_cloud);

//	std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.007); // 1cm
    ec.setMinClusterSize (200);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (filtered_cloud);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        PointCloudT::Ptr cloud_cluster (new PointCloudT);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
          cloud_cluster->points.push_back (filtered_cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
//	    std::stringstream ss;
//	    ss << "cloud_cluster_" << j << ".pcd";
//	    writer.write<PointT> (ss.str (), *cloud_cluster, false); //*
        j++;
    }
}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "identifier");

    IdentifierClass identifer_class_;

    ros::AsyncSpinner sp(4);
    sp.start();
    ros::waitForShutdown();

    return 0;
}
