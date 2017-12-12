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
#include <identification_msgs/IdentifierControlCommands.h>
#include <identification_msgs/IdentifierStates.h>
#include <identification_msgs/TactileProcessorStates.h>

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

#include "../include/model.h"
#include "../include/recog.h"

typedef pcl::PointXYZ            PointT;
typedef pcl::PointCloud<PointT>  PointCloudT;

typedef pcl::PointNormal         PointNT;
typedef pcl::PointCloud<PointNT> PointCloudNT;
typedef pcl::FPFHSignature33     FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerNT;


class IdentifierClass
{
public:
	IdentifierClass();
private:
	std::string  update_crof2marker_transform;
	std::string  used_marker_frame;
	int          num_of_recording_marker_frame;

	Model model;

	PointCloudT  pcl_cloud;
	PointCloudT  pcl_mean_filtered_cloud;
	PointCloudT  pcl_mean_and_passthrough_filtered_cloud;
	PointCloudT  pcl_mean_and_passthrough_and_statistic_filtered_cloud;
	PointCloudT  pcl_mean_and_passthrough_and_statistic_and_radiusoutlier_filtered_cloud;
	PointCloudT  pcl_mean_and_passthrough_and_statistic_and_radiusoutlier_and_downsample_filtered_cloud;

	bool pcl_xxx_filtered_cloud_valid_flag;

	PointCloudT  current_recognition_scene_cloud;

	identification_msgs::IdentifierControlCommands identifier_control_commands;
	identification_msgs::IdentifierStates          identifier_states;
	identification_msgs::TactileProcessorStates    tactile_processor_states;

	ros::Subscriber kinect2_point_sub;
	ros::Subscriber tactile_processor_states_sub;
	ros::Subscriber identifier_control_command_sub;

	ros::Publisher  identifier_states_pub;
	ros::Publisher  kinect2_filtered_point_pub;

	ros::Timer timer0;
	ros::Timer timer1;
	ros::Timer timer2;

	tf::TransformListener listener;
	tf::TransformBroadcaster tf_broadcaster;

	tf::Transform marker2base_transform; //Transform of marker frame to base frame
	tf::Transform cl2base_transform;     //Transform of camera_link frame to base frame

	void timer0EventCallback(const ros::TimerEvent& event);
	void timer1EventCallback(const ros::TimerEvent& event);
	void timer2EventCallback(const ros::TimerEvent& event);
	void tactileProcessorStateCallback(const identification_msgs::TactileProcessorStates& msg);
	void identifierControlCommandCallback(const identification_msgs::IdentifierControlCommands& msg);
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
	void setCL2BaseTransform(const std::string&   update_crof2marker_transform,
			                 const std::string&   marker_type,
							 const int&           num_of_recording_marker_frame,
			                 const tf::Transform& marker2base_transform,
							 	   tf::Transform& cl2base_transform);
	void transformPointCloudFromKinectFrameToWorldFrame(const PointCloudT & source_cloud,
									                          PointCloudT & transformed_cloud);
	void recognizeDesiredObjectFromScene(const PointCloudT & scene_cloud,
										 const identification_msgs::IdentifierControlCommands& identifier_control_commands,
										 const identification_msgs::TactileProcessorStates&    tactile_processor_states);
};

IdentifierClass::IdentifierClass()
{
	ros::NodeHandle nh("~");

	nh.param("update_crof2marker_transform", update_crof2marker_transform, std::string(""));
	nh.param("used_marker_frame", used_marker_frame, std::string(""));
	nh.param("num_of_recording_marker_frame", num_of_recording_marker_frame, 1);

	pcl_xxx_filtered_cloud_valid_flag = false;

	//=== Set constant transform marker frame to base frame ===//
	marker2base_transform.setOrigin(tf::Vector3(0.35 - 0.105, -0.375 + 0.105, 0.02));     //x, y, z
	marker2base_transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));//x, y, z, w

	setCL2BaseTransform(update_crof2marker_transform,
			            used_marker_frame,
						num_of_recording_marker_frame,
						marker2base_transform,
						cl2base_transform);

//	kinect2_point_sub              = nh.subscribe("/kinect2/sd/points", 1, &IdentifierClass::kinect2PointFilterCallback, this);
	kinect2_point_sub              = nh.subscribe("/camera/depth_registered/points", 1, &IdentifierClass::kinect2PointFilterCallback, this);
	tactile_processor_states_sub   = nh.subscribe("/tactile_processor_states", 1, &IdentifierClass::tactileProcessorStateCallback, this);
	identifier_control_command_sub = nh.subscribe("/identifier_control_commands", 1, &IdentifierClass::identifierControlCommandCallback, this);

	identifier_states_pub       = nh.advertise<identification_msgs::IdentifierStates>("/identifier_states", 1, true);
	kinect2_filtered_point_pub  = nh.advertise<sensor_msgs::PointCloud2>("/kinect2_filtered_points", 1, true);

	timer0 = nh.createTimer(ros::Duration(0.10), &IdentifierClass::timer0EventCallback, this);
	timer1 = nh.createTimer(ros::Duration(0.10), &IdentifierClass::timer1EventCallback, this);
	timer2 = nh.createTimer(ros::Duration(0.01), &IdentifierClass::timer2EventCallback, this);

//	ROS_WARN_STREAM("zyf_test: " << 1); //for debug
}

void IdentifierClass::timer0EventCallback(const ros::TimerEvent& event)
{
	if (pcl_xxx_filtered_cloud_valid_flag == true)
	{
//		viewTwoComparePointCloud(pcl_mean_filtered_cloud, pcl_mean_and_passthrough_and_statistic_and_radiusoutlier_filtered_cloud);
//	    viewSegmentPointCloud(pcl_mean_and_passthrough_and_statistic_and_radiusoutlier_filtered_cloud);

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

void IdentifierClass::timer1EventCallback(const ros::TimerEvent& event)
{
	recognizeDesiredObjectFromScene(pcl_mean_and_passthrough_and_statistic_and_radiusoutlier_filtered_cloud, identifier_control_commands, tactile_processor_states);
}

void IdentifierClass::timer2EventCallback(const ros::TimerEvent& event)
{
	identifier_states_pub.publish(identifier_states);

	//=== Broadcast the transforms of the marker frame and camera_link frame to base frame. ===//
	tf_broadcaster.sendTransform(tf::StampedTransform(marker2base_transform, ros::Time::now(), "base", "marker_board"));
	tf_broadcaster.sendTransform(tf::StampedTransform(cl2base_transform, ros::Time::now(), "base", "camera_link"));// For PS1080 PrimeSense
	if (identifier_states.potential_target_cluster_pose_valid_flag == true)
	{
		geometry_msgs::Pose pose = identifier_states.potential_target_cluster_pose_in_arm_base_frame;
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
		transform.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));
		tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "potential_target"));
	}
}

void IdentifierClass::tactileProcessorStateCallback(const identification_msgs::TactileProcessorStates& msg)
{
	tactile_processor_states = msg;
}

void IdentifierClass::identifierControlCommandCallback(const identification_msgs::IdentifierControlCommands& msg)
{
	identifier_control_commands = msg;
}

void IdentifierClass::kinect2PointFilterCallback(const sensor_msgs::PointCloud2& msg)
{
	//camera/depth_registered/points  = 640*480
	//kinect2/sd/points  = 512*424
	//kinect2/qhd/points = 960*540
	//kinect2/hd/points  = 1920*1080
	static int   point_number   = 640*480;
	static int   record_counter = 0;
	static int   record_counter_threshold = 30;
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
		pass.setFilterLimits (-0.6, 0.1);
		//pass.setFilterLimitsNegative (true);
		pass.filter (*filtered_cloud);

		*cloud = *filtered_cloud;

		pass.setInputCloud (cloud);
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (-0.15, 0.4);
		//pass.setFilterLimitsNegative (true);
		pass.filter (*filtered_cloud);

		pcl_mean_and_passthrough_filtered_cloud = *filtered_cloud;
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
			std::string path = ros::package::getPath("identifier");
			pcl::PCDWriter writer;
			writer.write<PointT> (path + "/PCD/model.pcd", *cloud_cluster, false);
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

		pcl::visualization::PointCloudColorHandlerCustom<PointT> segment_cloud_color_handler (cloud_cluster, r, g, b);//
		viewer.addPointCloud (cloud_cluster, segment_cloud_color_handler, cloud_actor_map_id);
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloud_actor_map_id);

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

	viewer.addCoordinateSystem (0.2, "sensor", 0);
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
	}
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
	std::cout << "PointCloud after filtering has: " << filtered_cloud->points.size ()  << " data points." << std::endl; //

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

void IdentifierClass::recognizeTargetBasedOnVisualPointCloud(const PointCloudT::Ptr source_cloud,
									                         const PointCloudT::Ptr target_cloud,
															       Eigen::Matrix4f& transformation,
															       float& fitness_score)
{
	PointCloudNT::Ptr object_source_cloud (new PointCloudNT);
	PointCloudNT::Ptr scene_target_cloud (new PointCloudNT);

	copyPointCloud (*source_cloud, *object_source_cloud);
	copyPointCloud (*target_cloud, *scene_target_cloud);

	Eigen::Matrix4f temp_transformation[2];
	float temp_fitness_score[2];

	//------------------------------------------------------------------------------------------//
	//-----------Select a transformation with a lower fitness score as estimation pose----------//
    //------------------------------------------------------------------------------------------//
	alignmentPrerejective(object_source_cloud, scene_target_cloud, temp_transformation[0], temp_fitness_score[0]);
	alignmentPrerejective(scene_target_cloud, object_source_cloud, temp_transformation[1], temp_fitness_score[1]);

	if (temp_fitness_score[0] <= temp_fitness_score[1])
	{
		transformation = temp_transformation[0];
		fitness_score  = temp_fitness_score[0];
	}
	else
	{
		transformation = temp_transformation[1].inverse();
		fitness_score  = temp_fitness_score[1];
	}

	PointCloudNT::Ptr object_aligned_source_cloud (new PointCloudNT);
	pcl::transformPointCloud (*object_source_cloud, *object_aligned_source_cloud, transformation);

	/*// Show alignment
	pcl::visualization::PCLVisualizer viewer("Alignment");
	viewer.addCoordinateSystem (0.2, "sensor", 0);

	viewer.addPointCloud (scene_target_cloud, ColorHandlerNT (scene_target_cloud, 0.0, 255.0, 0.0), "scene"); //green
	viewer.addPointCloud (object_source_cloud, ColorHandlerNT (object_source_cloud, 255.0, 255.0, 255.0), "object");  //white
	viewer.addPointCloud (object_aligned_source_cloud, ColorHandlerNT (object_aligned_source_cloud, 0.0, 0.0, 255.0), "object_aligned"); //blue
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
	}*/
}

void IdentifierClass::alignmentPrerejective(const PointCloudNT::Ptr source_cloud,
            			   	   	   	   	   	const PointCloudNT::Ptr target_cloud,
												  Eigen::Matrix4f& transformation,
												  float& fitness_score)
{
	// Align a rigid object to a scene with clutter and occlusions
	PointCloudNT::Ptr object (new PointCloudNT);
	PointCloudNT::Ptr object_aligned (new PointCloudNT);
	PointCloudNT::Ptr scene (new PointCloudNT);
	FeatureCloudT::Ptr object_features (new FeatureCloudT);
	FeatureCloudT::Ptr scene_features (new FeatureCloudT);

	*object = *source_cloud;
	*scene  = *target_cloud;

	// Downsample
//	pcl::console::print_highlight ("Downsampling...\n");
//	pcl::VoxelGrid<PointNT> grid;
	const float leaf = 0.005f;
	/*grid.setLeafSize (leaf, leaf, leaf);
	grid.setInputCloud (object);
	grid.filter (*object);
	grid.setInputCloud (scene);
	grid.filter (*scene);*/

	// Estimate normals
//	pcl::console::print_highlight ("Estimating normals...\n");
	pcl::NormalEstimationOMP<PointNT,PointNT> nest;
	nest.setRadiusSearch (0.02);
	nest.setInputCloud (object);
	nest.compute (*object);
	nest.setInputCloud (scene);
	nest.compute (*scene);

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
	align.setInlierFraction (0.50f); // Required inlier fraction for accepting a pose hypothesis
	{
	//	pcl::ScopeTime t("Alignment");
		align.align (*object_aligned);
	}

    fitness_score = (float) align.getFitnessScore(); //align.getInliers ().size ()/scene->size ();

	if (align.hasConverged ())
	{
		transformation = align.getFinalTransformation ();

/*	    // Print results
		printf ("\n");
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
		pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
		pcl::console::print_info ("\n");
		pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
		pcl::console::print_info ("\n");
*/		pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
//		pcl::console::print_info ("FitnessScore: %f\n", fitness_score);
		// Show alignment
	/*	pcl::visualization::PCLVisualizer viewer("Alignment");
		viewer.addCoordinateSystem (0.2, "sensor", 0);

		viewer.addPointCloud (scene, ColorHandlerNT (scene, 0.0, 255.0, 0.0), "scene");
		viewer.addPointCloud (object, ColorHandlerNT (object, 255.0, 255.0, 255.0), "object");
		viewer.addPointCloud (object_aligned, ColorHandlerNT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");


		viewer.addPointCloudNormals<PointNT, PointNT>(scene, scene, 30, 0.1, "scene_normals", 0);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "scene_normals");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "scene_normals");

		viewer.addPointCloudNormals<PointNT, PointNT>(object, object, 30, 0.1, "object_normals", 0);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.5, 0.0, "object_normals");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "object_normals");
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
		}*/
	}
	else
	{
		pcl::console::print_error ("Alignment failed!\n");
	}
}

void IdentifierClass::transformPointCloudFromKinectFrameToWorldFrame(const PointCloudT & source_cloud,
									                                       PointCloudT & transformed_cloud)
{
	//------------------------------------------------------------------------------------------//
	//------------Transform point cloud from kinect frame to world frame------------------------//
	//------------------------------------------------------------------------------------------//
	Eigen::Affine3d transform (Eigen::Affine3d::Identity ());
	try
	{
		tf::StampedTransform temp_transform;
		//=== For rosmsg /camera/depth/points ===//
//		listener.lookupTransform("base", "camera_depth_optical_frame", ros::Time(0), temp_transform);
		//=== For rosmsg /camera/depth_registered/points ===//
		listener.lookupTransform("base", "camera_rgb_optical_frame", ros::Time(0), temp_transform);
		tf::transformTFToEigen(temp_transform,transform);
	}
	catch(tf::LookupException& ex)
	{
		ROS_ERROR("Received an exception trying to lookupTransform from \"camera_depth_optical_frame\" to \"base\": %s", ex.what());
	}

	pcl::transformPointCloud (source_cloud, transformed_cloud, transform.cast<float>());
}

void IdentifierClass::setCL2BaseTransform(const std::string&   update_crof2marker_transform,
										  const std::string&   used_marker_frame,
										  const int&           num_of_recording_marker_frame,
        							      const tf::Transform& marker2base_transform,
										  	    tf::Transform& cl2base_transform)
{
	tf::StampedTransform transform;
	listener.waitForTransform("camera_rgb_optical_frame", "camera_link", ros::Time(0), ros::Duration(5));
	listener.lookupTransform("camera_rgb_optical_frame", "camera_link", ros::Time(0), transform);

	//=== Transform tf::StampedTransform to tf::Transform ===//
	tf::Transform cl2crof_transform;
	cl2crof_transform = transform;

	//=== set the crof2marker_transform ===//
	tf::Transform crof2marker_transform;
	tf::Quaternion q;
	if (update_crof2marker_transform == "start_online_once")
	{
		ROS_WARN_STREAM("update_crof2marker_transform: "<<update_crof2marker_transform);
		ROS_WARN_STREAM("used_marker_frame: "<<used_marker_frame);

		int num = num_of_recording_marker_frame;
		double roll, pitch, yaw;
		tf::Vector3 vector_origin_sum;
		vector_origin_sum.setZero();

		if (used_marker_frame == "ar_marker_4")
		{
			double roll_sum  = 0.0;
			double pitch_sum = 0.0;
			double yaw_sum   = 0.0;

			for (int i=0; i<num; i++)
			{
				listener.waitForTransform("ar_marker_4", "camera_rgb_optical_frame", ros::Time(0), ros::Duration(5));
				listener.lookupTransform("ar_marker_4", "camera_rgb_optical_frame", ros::Time(0), transform);

				vector_origin_sum = vector_origin_sum + transform.getOrigin();
				q = transform.getRotation();
				tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

				roll_sum  = roll_sum + roll;
				pitch_sum = pitch_sum + pitch;
				yaw_sum   = yaw_sum + yaw;
				usleep(100000); //0.1s
			}
			q.setRPY(roll_sum/num , pitch_sum/num, yaw_sum/num);
			crof2marker_transform.setOrigin(vector_origin_sum/num);
			crof2marker_transform.setRotation(q);
			ROS_WARN_STREAM("new_crof2marker_transform"); //for debug
			ROS_WARN_STREAM("roll, pitch and yaw: " << roll_sum/num<<","<<pitch_sum/num<<","<<yaw_sum/num); //for debug
			ROS_WARN_STREAM("x, y and z: " << vector_origin_sum.getX()/num<<","<<vector_origin_sum.getY()/num<<","<<vector_origin_sum.getZ()/num); //for debug
		}
		else if (used_marker_frame == "ar_marker_9-17")
		{
			tf::StampedTransform transform_10, transform_12, transform_13, transform_14, transform_16;
			tf::Vector3 vector_x, vector_y, vector_z;
			vector_x.setZero();
			vector_y.setZero();
			vector_z.setZero();

			for (int i=0; i<num; i++)
			{
				listener.waitForTransform("camera_rgb_optical_frame", "ar_marker_10", ros::Time(0), ros::Duration(5));
				listener.lookupTransform("camera_rgb_optical_frame", "ar_marker_10", ros::Time(0), transform_10);
				listener.waitForTransform("camera_rgb_optical_frame", "ar_marker_12", ros::Time(0), ros::Duration(5));
				listener.lookupTransform("camera_rgb_optical_frame", "ar_marker_12", ros::Time(0), transform_12);
				listener.waitForTransform("camera_rgb_optical_frame", "ar_marker_13", ros::Time(0), ros::Duration(5));
				listener.lookupTransform("camera_rgb_optical_frame", "ar_marker_13", ros::Time(0), transform_13);
				listener.waitForTransform("camera_rgb_optical_frame", "ar_marker_14", ros::Time(0), ros::Duration(5));
				listener.lookupTransform("camera_rgb_optical_frame", "ar_marker_14", ros::Time(0), transform_14);
				listener.waitForTransform("camera_rgb_optical_frame", "ar_marker_16", ros::Time(0), ros::Duration(5));
				listener.lookupTransform("camera_rgb_optical_frame", "ar_marker_16", ros::Time(0), transform_16);
				usleep(100000); //0.1s

				vector_x = vector_x + (transform_10.getOrigin() - transform_16.getOrigin());
				vector_y = vector_y + (transform_12.getOrigin() - transform_14.getOrigin());
				vector_origin_sum = vector_origin_sum + transform_13.getOrigin();
			}
			vector_x = vector_x.normalized();
			vector_y = vector_y.normalized();
			vector_z = vector_x.cross(vector_y);
			vector_z = vector_z.normalized();

			tf::Matrix3x3 matr;
			matr.setValue(vector_x.getX(), vector_y.getX(), vector_z.getX(),
						  vector_x.getY(), vector_y.getY(), vector_z.getY(),
						  vector_x.getZ(), vector_y.getZ(), vector_z.getZ());
			matr.getRotation(q);

			transform.setOrigin(vector_origin_sum/num);
			transform.setRotation(q);
			crof2marker_transform = transform.inverse();

			matr.setRotation(crof2marker_transform.getRotation());
			matr.getRPY(roll, pitch, yaw);

			ROS_WARN_STREAM("new_crof2marker_transform"); //for debug
			ROS_WARN_STREAM("roll, pitch and yaw: " << roll<<","<<pitch<<","<<yaw); //for debug
			ROS_WARN_STREAM("x, y and z: " << crof2marker_transform.getOrigin().getX()<<","<<crof2marker_transform.getOrigin().getY()<<","<<crof2marker_transform.getOrigin().getZ()); //for debug
		}
	}
	else if (update_crof2marker_transform == "use_default_value")
	{
		ROS_WARN_STREAM("update_crof2marker_transform: "<<update_crof2marker_transform);
		q.setRPY(-2.18019,-0.0617434,-0.138399);
		crof2marker_transform.setRotation(q);
		crof2marker_transform.setOrigin(tf::Vector3(0.0510037,-0.49952,0.572442)); //x, y, z
	}
	else
	{
		ROS_FATAL("The parameter value of update_crof2marker_transform is illegal.");
	    exit (-1);
	}

	//=== Calculate transform camera_link frame to base frame ===//
	cl2base_transform = marker2base_transform * crof2marker_transform * cl2crof_transform;
}

void IdentifierClass::recognizeDesiredObjectFromScene(const PointCloudT& scene_cloud,
									 	 	 	 	  const identification_msgs::IdentifierControlCommands& identifier_control_commands,
													  const identification_msgs::TactileProcessorStates&    tactile_processor_states)
{
	static bool history_start_recognition_command = false;
	static bool current_start_recognition_command = false;
	history_start_recognition_command = current_start_recognition_command;
	current_start_recognition_command = identifier_control_commands.start_recognition_command;

	if((history_start_recognition_command == false) &&
	   (current_start_recognition_command == true))
	{
		//------------------------------------------------------------------------------------------//
		//------------------------------Start recognition process...--------------------------------//
		//------------------------------------------------------------------------------------------//
		ROS_INFO_STREAM("Start recognition process...");

        //=== Waiting until the flag is true. ===//
		ROS_INFO_STREAM("Waiting until pcl_xxx_filtered_cloud_valid_flag is equal to true...");
		while (pcl_xxx_filtered_cloud_valid_flag == false)	usleep(100000);//0.1s
		//------------------------------------------------------------------------------------------//
		//-------------------------Store scene cloud used for recognition---------------------------//
		//------------------------------------------------------------------------------------------//
		current_recognition_scene_cloud	 = scene_cloud;

		//------------------------------------------------------------------------------------------//
		//------------------------------segment point cloud-----------------------------------------//
		//------------------------------------------------------------------------------------------//
		PointCloudT::Ptr cloud (new PointCloudT);
		PointCloudT::Ptr plane_cloud (new PointCloudT);
		PointCloudT::Ptr filtered_cloud (new PointCloudT);
		std::vector<pcl::PointIndices> cluster_indices;

		ROS_INFO_STREAM("Start segment process...");

		ros::WallTime start_time = ros::WallTime::now();

		*cloud = current_recognition_scene_cloud;

		segmentPointCloud(cloud, plane_cloud, filtered_cloud, cluster_indices);

		double time = ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();

		ROS_INFO_STREAM("Finish Segment work and time used for segmenting: " <<time);

		//=== Transform point cloud to world coordinate ===//
		transformPointCloudFromKinectFrameToWorldFrame(*filtered_cloud, *filtered_cloud);
		filtered_cloud->header.frame_id = "base";

		//------------------------------------------------------------------------------------------//
		//---------------------------------Extract clustered clouds --------------------------------//
		//------------------------------------------------------------------------------------------//
		std::vector<PointCloudT>     clustered_cloud;
		std::vector<Eigen::Matrix4f> transformation;
		std::vector<float>           fitness_score;
		std::vector<int>             source_file_index;

		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			PointCloudT::Ptr cloud_cluster (new PointCloudT);
			for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
				cloud_cluster->points.push_back (filtered_cloud->points[*pit]); //*
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;

			clustered_cloud.push_back(*cloud_cluster);
		}

		//------------------------------------------------------------------------------------------//
		//---------------------Estimate pose and calculate the fitness score------------------------//
		//------------------------------------------------------------------------------------------//
		PointCloudT::Ptr desired_object_cloud (new PointCloudT);
		PointCloudT::Ptr target_cloud (new PointCloudT);

		std::string model_file_path = ros::package::getPath("knowledge_base") + "/models/"+ identifier_control_commands.desired_object_name;
		model = Model(model_file_path);

		for(int i=0; i<clustered_cloud.size(); i++)
		{
			*target_cloud = clustered_cloud[i];

			int num_submodel = model.raw.size();

			Eigen::Matrix4f temp_transformation[num_submodel];
			float temp_fitness_score[num_submodel];
			float temp_percentage[num_submodel];
			bool  temp_valid_flag[num_submodel];

            //=== Set initial parameter values. ===//
			for (int j=0; j<num_submodel; j++)
			{
				temp_fitness_score[j] = 1.0;
				temp_percentage[j]    = 0.0;
				temp_valid_flag[j]	  = false;
			}
			//=== Estimate target pose ===//
			for (int j=0; j<num_submodel; j++)
			{
				*desired_object_cloud = *model.raw[j];

				temp_percentage[j] = (float) clustered_cloud[i].size()/desired_object_cloud->size(); //Both clouds have been filtered by using pcl::VoxelGrid with 0.005 leaf in camera space.

				if (fabs(temp_percentage[j] - 1) < 0.1)
					temp_valid_flag[j] = true;

				if (temp_valid_flag[j] == true)
					recognizeTargetBasedOnVisualPointCloud(desired_object_cloud, target_cloud, temp_transformation[j], temp_fitness_score[j]);

				ROS_WARN_STREAM("temp_fitness_score_"<<j<<": "<<temp_fitness_score[j]); //for debug
				ROS_WARN_STREAM("temp_percentage_"<<j<<": "<<temp_percentage[j]); //for debug
			}

			float min_value       = 1.0;
			int   min_value_index = 0;
			for (int j=0; j<num_submodel; j++)
			{
				if (temp_valid_flag[j] == true)
				{
					if (min_value > temp_fitness_score[j])
					{
						min_value       = temp_fitness_score[j];
						min_value_index = j;
					}
				}
			}
			transformation.push_back (temp_transformation[min_value_index]);
			fitness_score.push_back (temp_fitness_score[min_value_index]);
			source_file_index.push_back (min_value_index);
			ROS_WARN_STREAM("source_file_index_"<<i<<": "<<source_file_index[i]); //for debug
		}

		//------------------------------------------------------------------------------------------//
		//---Determine potential target clusters based on fitness scores and clustered cloud sizes--//
		//------------------------------------------------------------------------------------------//
		// Determine validity of clustered clouds
		std::vector<bool>  valid_flag;

		for (int i=0; i<clustered_cloud.size(); i++)
		{
			*desired_object_cloud = *model.raw[source_file_index[i]];

			float percentage = (float) clustered_cloud[i].size()/desired_object_cloud->size(); //Both clouds have been filtered by using pcl::VoxelGrid with 0.005 leaf in camera space.

			ROS_INFO_STREAM("percentage_"<<i<<": " <<percentage<<"; "<<"fitness_score_"<<i<<": "<<fitness_score[i]);
			if ((fabs(percentage - 1) < 0.1) && (fitness_score[i] < 0.00005))//0.000015
				valid_flag.push_back(true);
			else
				valid_flag.push_back(false);
		}

		// Sort clustered clouds based on valid fitness scores from low to high
		std::vector<PointCloudT>     potential_target_clustered_cloud;
		std::vector<Eigen::Matrix4f> potential_target_transformation;
		std::vector<float>           potential_target_fitness_score;
		std::vector<int>             potential_target_source_file_index;
		float min_fitness_score;
		int   min_fitness_score_index = 0;

		for (int i=0; i<clustered_cloud.size(); i++)
		{
			min_fitness_score = 1.0;
			for (int j=0; j<clustered_cloud.size(); j++)
			{
				if ((valid_flag[j] == true) && (min_fitness_score > fitness_score[j]))
				{
					min_fitness_score       = fitness_score[j];
					min_fitness_score_index = j;
				}
			}

			if (valid_flag[min_fitness_score_index] == true)
			{
				potential_target_clustered_cloud.push_back (clustered_cloud[min_fitness_score_index]);
				potential_target_transformation.push_back (transformation[min_fitness_score_index]);
				potential_target_fitness_score.push_back (fitness_score[min_fitness_score_index]);
				potential_target_source_file_index.push_back (source_file_index[min_fitness_score_index]);
				valid_flag[min_fitness_score_index] = false;
			}
		}

		ROS_INFO_STREAM("num_potential_target: " <<potential_target_clustered_cloud.size());

		identifier_states.num_potential_target = potential_target_clustered_cloud.size();
		identifier_states.num_potential_target_valid_flag = true;

		//------------------------------------------------------------------------------------------//
		//----------------------Recognize potential target clusters---------------------------------//
		//------------------------------------------------------------------------------------------//

		for (int i=0; i<potential_target_clustered_cloud.size(); i++)
		{
			if (identifier_control_commands.start_recognition_command == false) break;
			//------------------------------------------------------------------------------------------//
			//---------------convert Eigen::Matrix4f to geometry_msgs::Pose and output------------------//
			//------------------------------------------------------------------------------------------//
			geometry_msgs::Pose pose;
			Eigen::Affine3f  transform;
			transform.matrix() = potential_target_transformation[i];
			//std::cout << transform.matrix() << std::endl;
			/////////////////////////////////////////////////
			//Let z axis parallel to z axis of marker board//
			/////////////////////////////////////////////////
			Eigen::Matrix4f mf = potential_target_transformation[i];
			tf::Vector3 v1, v2, v3;
			v1.setValue(static_cast<double>(mf(0,2)), static_cast<double>(mf(1,2)), static_cast<double>(mf(2,2)));
			v2 = marker2base_transform.getBasis().getColumn(2);
			v3 = v1.cross(v2);
			v1.normalize();
			v2.normalize();
			v3.normalize();
			float anglef = v1.angle(v2);
			Eigen::Matrix3f rot = mf.block<3, 3>(0, 0);
			Eigen::Vector3f axisf (v3.x(), v3.y(), v3.z());
			axisf = rot.inverse() * axisf;
			axisf.normalize();
			ROS_WARN_STREAM("Rotate angle (unit: degree): " << anglef*180/M_PI); //for debug
			transform.rotate(Eigen::AngleAxisf(anglef, axisf));
			std::cout << "Transform matrix of potential target..." << std::endl;
			std::cout << transform.matrix() << std::endl;
			/////////////////////////////////////////////////
			tf::poseEigenToMsg(transform.cast<double>(), pose);
			identifier_states.potential_target_source_file_index = potential_target_source_file_index[i];
			identifier_states.potential_target_cluster_pose_in_arm_base_frame = pose;
			identifier_states.potential_target_cluster_pose_valid_flag = true;

			//------------------------------------------------------------------------------------------//
			//----Acquire contact points and contact features for the i-th potential target cluster-----//
			//------------------------------------------------------------------------------------------//
			PointCloudNT contact_point_cloud[3];
			bool   history_contact_valid_flag[3];
			bool   history_contact_featrue_valid_flag;
			std::vector<float> contact_feature;

			//=== Waiting until grasp process command is true. ===//
			ROS_INFO_STREAM("Waiting until grasp process command is true... ");
			while (identifier_control_commands.grasp_process_command == false)
			{
				if (identifier_control_commands.start_recognition_command == false) break;
				usleep(10000);//0.01s
			}

			ROS_INFO_STREAM("Waiting until grasp process command is fals... ");
			while (identifier_control_commands.grasp_process_command == true)
			{
				ROS_INFO_ONCE("Grasp process...");
				if (identifier_control_commands.start_recognition_command == false) break;
				//------------------------------------------------------------------------------------------//
				//----Detect contact events, transform contact points and store them as point cloud---------//
				//------------------------------------------------------------------------------------------//
				for (int j=0; j<3; j++)
				{
					if (tactile_processor_states.fingertip_contact_pose_valid_flag[j] == false)
					{
						history_contact_valid_flag[j] = false;
					}

					if (history_contact_valid_flag[j] == false)
					{
						if (tactile_processor_states.fingertip_contact_pose_valid_flag[j] == true)
						{
							history_contact_valid_flag[j] = true;

							//=== Transform contact points and normal vectors to world frame and store them. ===//
							try
							{
								tf::StampedTransform transform;
								if (j==0) listener.lookupTransform("potential_target", "sdh_thumb_tactile_link", ros::Time(0), transform);
								if (j==1) listener.lookupTransform("potential_target", "sdh_finger1_tactile_link", ros::Time(0), transform);
								if (j==2) listener.lookupTransform("potential_target", "sdh_finger2_tactile_link", ros::Time(0), transform);

								tf::Transform transform_A;
								transform_A = transform;

								geometry_msgs::Pose pose;
								pose = tactile_processor_states.fingertip_contact_pose[j];

								tf::Transform transform_B;
								transform_B.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
								transform_B.setRotation(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

								tf::Transform transform_C;
								transform_C = transform_A * transform_B;

								PointNT contact_point;
								contact_point.x = transform_C.getOrigin().getX();
								contact_point.y = transform_C.getOrigin().getY();
								contact_point.z = transform_C.getOrigin().getZ();
								contact_point.normal_x = transform_C.getBasis().getColumn(0).getX();
								contact_point.normal_y = transform_C.getBasis().getColumn(0).getY();
								contact_point.normal_z = transform_C.getBasis().getColumn(0).getZ();

								contact_point_cloud[j].header.frame_id = "potential_target";
								contact_point_cloud[j].points.push_back(contact_point);
								contact_point_cloud[j].width  = contact_point_cloud[j].points.size();
								contact_point_cloud[j].height = 1;
								contact_point_cloud[j].is_dense = true;
							}
							catch(tf::LookupException& ex)
							{
								ROS_ERROR("Received an exception trying to lookupTransform from \"sdh_***_tactile_link\" to \"base\": %s", ex.what());
							}
						}
					}
				}
				//------------------------------------------------------------------------------------------//
				//----------------Detect contact feature valid event and then store it----------------------//
				//------------------------------------------------------------------------------------------//
				if (tactile_processor_states.contact_feature_valid_flag == false)
				{
					history_contact_featrue_valid_flag = false;
				}
				if (history_contact_featrue_valid_flag == false)
				{
					if (tactile_processor_states.contact_feature_valid_flag == true)
					{
						usleep(500000);//0.5s
						contact_feature.push_back(tactile_processor_states.contact_feature);
						ROS_WARN_STREAM("contact feature:"<<tactile_processor_states.contact_feature);
					}
				}
				usleep(10000);//0.01s
			}
			identifier_states.potential_target_cluster_pose_valid_flag = false;
			ROS_INFO_STREAM("Grasp process is finished.");

			//------------------------------------------------------------------------------------------//
			//------------Transform point cloud from base frame to potential_target frame---------------//
			//------------------------------------------------------------------------------------------//
			pcl::transformPointCloud (potential_target_clustered_cloud[i], potential_target_clustered_cloud[i], transform.inverse());
			potential_target_clustered_cloud[i].header.frame_id = "potential_target";
			//------------------------------------------------------------------------------------------//
			//------------------------------------Save experimental data--------------------------------//
			//------------------------------------------------------------------------------------------//
			std::stringstream ss;
			ss << i;
			std::string path = ros::package::getPath("identifier") + "/ExpData/" + identifier_control_commands.experiment_label + ss.str ();
			pcl::PCDWriter writer;
			writer.write<PointT>  (path + "_target.pcd", potential_target_clustered_cloud[i], false);

			for(int j=0; j<3; j++)
			{
				PointCloudNT::Ptr source_cloud (new PointCloudNT);
				PointCloudT::Ptr target_cloud (new PointCloudT);

				*source_cloud = contact_point_cloud[j];
				copyPointCloud (*source_cloud, *target_cloud);

				if (contact_point_cloud[j].size() > 0)
				{
					ss.str("");
					ss << j;
					writer.write<PointNT> (path + "_finger_" + ss.str () + "_NT.pcd", contact_point_cloud[j], false);
					writer.write<PointT>  (path + "_finger_" + ss.str () + "_T.pcd", *target_cloud, false);
				}
				else
					ROS_INFO_STREAM("Contact point cloud of finger_"<<j<<" is empty.");
			}

			std::ofstream outfile ((path + "_contact_feature.txt").c_str());
			vector<float>::iterator t;
			for(t=contact_feature.begin(); t!=contact_feature.end(); t++)
				outfile<<*t<<'\n';
			outfile.close();

			std::ofstream temp_outfile ((ros::package::getPath("identifier") + "/ExpData/" + "contact_feature.txt").c_str());
			for(t=contact_feature.begin(); t!=contact_feature.end(); t++)
				temp_outfile<<*t<<'\n';
			temp_outfile.close();
			//------------------------------------------------------------------------------------------//
			//--------------------Re-estimate the i-th potential target cluster pose--------------------//
			//------------------------------------------------------------------------------------------//
			//--------Extract the 2-D shape features of specific layers from fused point cloud----------//
			//------------------------------------------------------------------------------------------//
			//-------Computing similarity with desired object feature vector or object database---------//
			//------------------------------------------------------------------------------------------//
			//-------------------------------------Fuse decision----------------------------------------//
			//------------------------------------------------------------------------------------------//
			std::string database_file_path = ros::package::getPath("knowledge_base") + "/models/database";
			Recog recog = Recog(database_file_path);
			bool flag = recog.isRecognitionSuccess(model.id, potential_target_clustered_cloud[i], contact_point_cloud[0], contact_point_cloud[1], contact_point_cloud[2], contact_feature);

			writer.write<PointT>  (path + "_target_aligned.pcd", potential_target_clustered_cloud[i], false);

			if (flag == true)
			{
				identifier_states.recognition_success_flag  = true;
				break;
			}
		}
		identifier_states.recognition_finished_flag = true;
		ROS_INFO_STREAM("Finish recognition process.");
	}
	else if(identifier_control_commands.start_recognition_command == false)
	{
		// Reset identifier states
		identifier_states.recognition_finished_flag = false;
		identifier_states.recognition_success_flag  = false;
		identifier_states.num_potential_target_valid_flag = false;
		identifier_states.potential_target_cluster_pose_valid_flag = false;
		identifier_states.num_potential_target = 0;
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
