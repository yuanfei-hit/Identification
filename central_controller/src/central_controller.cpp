/********************************************************************************
 * central_controller.cpp
 *
 *  Max stable frequency almost equal to 200 Hz.
 *
 *  Created on: 12 Jun 2017
 *      Author: yuanfei
********************************************************************************/

#include <boost/date_time.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>//For sensor_msgs::JointState
#include <geometry_msgs/Pose.h>    //For geometry_msgs::Pose
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

#include <identification_msgs/PlannerControlCommands.h>
#include <identification_msgs/PlannerStates.h>
#include <identification_msgs/TactileProcessorControlCommands.h>
#include <identification_msgs/TactileProcessorStates.h>
#include <identification_msgs/IdentifierControlCommands.h>
#include <identification_msgs/IdentifierStates.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "../include/model.h"


class CentralController
{
public:
	CentralController();
private:

	std::string desired_object;
	std::string experiment_label;
	int num_loop;
	Model model;
	Database database;

	geometry_msgs::Pose g2b_home;

	identification_msgs::PlannerStates                   planner_states;
	identification_msgs::TactileProcessorStates          tactile_processor_states;
	identification_msgs::IdentifierStates                identifier_states;

	identification_msgs::PlannerControlCommands          planner_control_commands;
	identification_msgs::TactileProcessorControlCommands tactile_processor_control_commands;
	identification_msgs::IdentifierControlCommands       identifier_control_commands;

	tf::TransformListener listener;

	ros::Subscriber planner_state_sub;
	ros::Subscriber tactile_processor_state_sub;
    ros::Subscriber identifier_state_sub;

	ros::Publisher  planner_control_command_pub;
    ros::Publisher  tactile_processor_control_command_pub;
    ros::Publisher  identifier_control_command_pub;

	ros::Timer timer;

	void plannerStatesCallback(const identification_msgs::PlannerStates& msg);
	void tactileProcessorStatesCallback(const identification_msgs::TactileProcessorStates& msg);
	void identifierStatesCallback(const identification_msgs::IdentifierStates& msg);
	void timerEventCallback(const ros::TimerEvent& event);
	void setPose(const tf::Quaternion& q,
			     const tf::Vector3&    p,
				 geometry_msgs::Pose&  pose);
	void getGripperToBasePose(const geometry_msgs::Pose&              object2base_pose,
								    std::vector<geometry_msgs::Pose>& gripper2base_pose,
									std::vector<bool>&                enable_gripper_control_flag,
									std::vector<bool>&                enable_clamping_mode);

	void transformPose(const geometry_msgs::Pose& old2base_pose,
                       const geometry_msgs::Pose& new2old_pose,
				             geometry_msgs::Pose& new2base_pose);
	bool boolArmReachDesiredPose(const geometry_msgs::Pose& gripper2base_pose,
			                     const double& position_threshold);
	bool boolAllFingerContactOccurOrReachEndPose(const identification_msgs::PlannerStates&          planner_states,
	                                             const identification_msgs::TactileProcessorStates& tactile_processor_states);
	void generateCommands(const identification_msgs::PlannerStates&                   planner_states,
						  const identification_msgs::TactileProcessorStates&          tactile_processor_states,
						  const identification_msgs::IdentifierStates&                identifier_states,
						        identification_msgs::PlannerControlCommands&          planner_control_commands,
						        identification_msgs::TactileProcessorControlCommands& tactile_processor_control_commands,
						        identification_msgs::IdentifierControlCommands&       identifier_control_commands);
};

CentralController::CentralController()
{
	ros::NodeHandle nh("~");

	nh.param("desired_object", desired_object, std::string(""));
	nh.param("experiment_label", experiment_label, std::string(""));

	if (desired_object=="")
	{
		ROS_FATAL("Missing desired object info in launch file");
	  	exit (-1);
	}

	num_loop = 0;

	std::string path = ros::package::getPath("knowledge_base") + "/models/"+ desired_object;
	model = Model(path);
	database = Database(ros::package::getPath("knowledge_base") + "/models/database");
//	database.setHandParameter(0.033, 0.03, 0.03);
	//=========Set initial gripper pose========//
	tf::Quaternion q;
	tf::Vector3    p;
    q.setValue(0.212619457254, 0.674385317701, -0.67438105178, 0.212620804879);
	p.setValue(0.576113174177, -0.00999857647083, 0.494038047031);
	setPose(q.normalized(), p, g2b_home);

	planner_state_sub           = nh.subscribe("/planner_states", 1, &CentralController::plannerStatesCallback, this);
	tactile_processor_state_sub = nh.subscribe("/tactile_processor_states", 1, &CentralController::tactileProcessorStatesCallback, this);
	identifier_state_sub        = nh.subscribe("/identifier_states", 1, &CentralController::identifierStatesCallback, this);

	planner_control_command_pub           = nh.advertise<identification_msgs::PlannerControlCommands>("/planner_control_commands", 1, true);
	tactile_processor_control_command_pub = nh.advertise<identification_msgs::TactileProcessorControlCommands>("/tactile_processor_control_commands", 1, true);
	identifier_control_command_pub        = nh.advertise<identification_msgs::IdentifierControlCommands>("/identifier_control_commands", 1, true);

	planner_control_commands.new_pose_command     = false;
	planner_control_commands.hand_grasp_command   = false;
	planner_control_commands.hand_release_command = true;
	planner_control_command_pub.publish(planner_control_commands);

	tactile_processor_control_commands.reset_tactile_processor_command = true;
	tactile_processor_control_command_pub.publish(tactile_processor_control_commands);

	identifier_control_commands.start_recognition_command = false;
	identifier_control_command_pub.publish(identifier_control_commands);

	sleep(5);

	timer = nh.createTimer(ros::Duration(1.00), &CentralController::timerEventCallback, this);
//	ROS_WARN_STREAM("zyf_test: " << time); //for debug
}

void CentralController::plannerStatesCallback(const identification_msgs::PlannerStates& msg)
{
	planner_states = msg;
}

void CentralController::tactileProcessorStatesCallback(const identification_msgs::TactileProcessorStates& msg)
{
	tactile_processor_states = msg;
}

void CentralController::identifierStatesCallback(const identification_msgs::IdentifierStates& msg)
{
	identifier_states = msg;
}

void CentralController::timerEventCallback(const ros::TimerEvent& event)
{
	generateCommands(planner_states, tactile_processor_states, identifier_states,
					 planner_control_commands, tactile_processor_control_commands, identifier_control_commands);
}

void CentralController::setPose(const tf::Quaternion&      q,
			                    const tf::Vector3&         p,
				                      geometry_msgs::Pose& pose)
{
	pose.orientation.x = q.getX();
	pose.orientation.y = q.getY();
	pose.orientation.z = q.getZ();
	pose.orientation.w = q.getW();
	pose.position.x = p.getX();
	pose.position.y = p.getY();
	pose.position.z = p.getZ();
}

void CentralController::getGripperToBasePose(const geometry_msgs::Pose&              object2base_pose,
							                       std::vector<geometry_msgs::Pose>& gripper2base_pose,
												   std::vector<bool>&                enable_gripper_control_flag,
												   std::vector<bool>&                enable_clamping_mode)
{
//	tf::StampedTransform transform;
//	listener.lookupTransform("base", "camera_rgb_optical_frame", ros::Time(0), transform);

//	double angle = atan2((object2base_pose.position.y - transform.getOrigin().getY()), (object2base_pose.position.x - transform.getOrigin().getX()));
	//------------------------------------------------------------------------------------------//
	//-----------------------------Calculate gripper desired pose-------------------------------//
	//------------------------------------------------------------------------------------------//
	ROS_INFO_STREAM("Calculate gripper desired pose."); //for debug

	geometry_msgs::Pose g2b_initial_a;
	geometry_msgs::Pose g2b_initial_b;
	geometry_msgs::Pose g2b_start_a;
	geometry_msgs::Pose g2b_start_b;
	geometry_msgs::Pose g2g_next;
	geometry_msgs::Pose g2b_next;
	geometry_msgs::Pose g2b_current;

	tf::Quaternion q;
	tf::Vector3    p;
    //== Set g2b_initial_* pose. ===//
	q.setRPY(M_PI, M_PI_2, 0.0); //roll Angle around X, pitch Angle around Y, yaw Angle around Z
	p.setValue(0.6, -0.4 , 0.40);//Z vale must be larger than 0.06m and smaller than object height minus 0.033m.
	setPose(q, p, g2b_initial_a);

	q.setRPY(M_PI, M_PI_2, M_PI_2);
	p.setValue(0.5, -0.5, 0.40);
	setPose(q, p, g2b_initial_b);

/*	//=== For acquiring tactile feature ===//
	gripper2base_pose.push_back(g2b_initial_a);
	enable_gripper_control_flag.push_back(false);
	enable_clamping_mode.push_back(false);

	g2b_next = g2b_initial_a;
	g2b_next.position.z = object2base_pose.position.z + 0.11;
	for (int i=0; i<20; i++)
	{
		gripper2base_pose.push_back(g2b_next);
		enable_gripper_control_flag.push_back(true);
		enable_clamping_mode.push_back(true);
	}*/

/*	//== Test grasp space range. ===//
	q.setRPY(0.0, 0.0, 0.0);
	p.setValue(0.0, 0.0, -0.1);
	setPose(q, p, g2g_next);
	transformPose(g2b_initial_a, g2g_next, g2b_start_a);

	q.setRPY(0.0, 0.0, 0.0);
	p.setValue(0.0, 0.0, -0.1);
	setPose(q, p, g2g_next);
	transformPose(g2b_initial_b, g2g_next, g2b_start_b);
*/

	//=== Set g2b_start_* pose. ===//
    double search_distance = 0.03-0.01; //Its value should be determined based on point cloud.
    double compensation_distance = 0.03-0.015;
	double ux = 0.7;
	double lx = 0.5 + search_distance;
	double uy = -0.4 - search_distance;
	double ly = -0.6;

	g2b_start_a = g2b_initial_a;
	g2b_start_b = g2b_initial_b;

	if (object2base_pose.position.x < lx)
	{
		g2b_start_a.position.x = lx;
		g2b_start_b.position.x = lx - compensation_distance;
	}
	else if (object2base_pose.position.x > ux)
	{
		g2b_start_a.position.x = ux;
		g2b_start_b.position.x = ux - compensation_distance;
	}
	else
	{
		g2b_start_a.position.x = object2base_pose.position.x - 0.02;
		g2b_start_b.position.x = object2base_pose.position.x - compensation_distance - 0.03;
	}

	if (object2base_pose.position.y < ly)
	{
		g2b_start_a.position.y = ly + compensation_distance;
		g2b_start_b.position.y = ly;
	}
	else if (object2base_pose.position.y > uy)
	{
		g2b_start_a.position.y = uy + compensation_distance;
		g2b_start_b.position.y = uy;
	}
	else
	{
		g2b_start_a.position.y = object2base_pose.position.y + compensation_distance;
		g2b_start_b.position.y = object2base_pose.position.y - 0.02;
	}

	gripper2base_pose.push_back(g2b_home);
	enable_gripper_control_flag.push_back(false);
	enable_clamping_mode.push_back(false);

	gripper2base_pose.push_back(g2b_initial_a);
	enable_gripper_control_flag.push_back(false);
	enable_clamping_mode.push_back(false);

	gripper2base_pose.push_back(g2b_start_a);
	enable_gripper_control_flag.push_back(false);
	enable_clamping_mode.push_back(false);

	g2b_current = g2b_start_a;

	int index = database.search(model.id);
	std::vector<float> height = database.slice_heights[index];

	if (database.similar_stiffness_model_ids[index].empty())
	{
		//=== For acquiring tactile feature ===//
		g2b_next = g2b_current;
		g2b_next.position.z = object2base_pose.position.z + 0.11;

		gripper2base_pose.push_back(g2b_next);
		enable_gripper_control_flag.push_back(false);
		enable_clamping_mode.push_back(false);

		for(int i = 0; i <= 6; i ++)
		{
			g2b_next.position.y = object2base_pose.position.y + 0.015;

			gripper2base_pose.push_back(g2b_next);
			enable_gripper_control_flag.push_back(true);
			enable_clamping_mode.push_back(true);
		}

		gripper2base_pose.push_back(g2b_initial_a);
		enable_gripper_control_flag.push_back(false);
		enable_clamping_mode.push_back(false);

		gripper2base_pose.push_back(g2b_home);
		enable_gripper_control_flag.push_back(false);
		enable_clamping_mode.push_back(false);
	}
	else
	{
		for (int i = 0; i < height.size(); i ++)
		{
			if (height[i] < 0.11)
				height[i] = height[i] + 0.033;
			else
				height[i] = height[i] - 0.033;
		}

		for(int i = 0; i < height.size(); i ++)
		{
			for(int j = 0; j <= (search_distance/0.005); j ++)
			{
				q.setRPY(0.0, 0.0, 0.0);
				if (j == 0)
					p.setValue(0.0, 0.0, 0.0);
				else
					p.setValue(0.0, pow(-1.0, (double)i)*0.005, 0.0);
				setPose(q, p, g2g_next);
				transformPose(g2b_current, g2g_next, g2b_next);
				//=== According to model layer information to adjust the pz value of g2b_next. ===//
				g2b_next.position.z = object2base_pose.position.z + height[i];

				g2b_current = g2b_next;

				gripper2base_pose.push_back(g2b_next);
				enable_gripper_control_flag.push_back(true);
				enable_clamping_mode.push_back(false);
			}
		}

		gripper2base_pose.push_back(g2b_initial_a);
		enable_gripper_control_flag.push_back(false);
		enable_clamping_mode.push_back(false);

		gripper2base_pose.push_back(g2b_home);
		enable_gripper_control_flag.push_back(false);
		enable_clamping_mode.push_back(false);

		gripper2base_pose.push_back(g2b_initial_b);
		enable_gripper_control_flag.push_back(false);
		enable_clamping_mode.push_back(false);

		gripper2base_pose.push_back(g2b_start_b);
		enable_gripper_control_flag.push_back(false);
		enable_clamping_mode.push_back(false);

		g2b_current = g2b_start_b;

		bool flag = false;

		for(int i = 0; i < height.size(); i ++)
		{
			for(int j = 0; j <= (search_distance/0.005); j ++)
			{
				q.setRPY(0.0, 0.0, 0.0);
				if (j == 0)
					p.setValue(0.0, 0.0, 0.0);
				else
					p.setValue(0.0, pow(-1.0, (double)i)*0.005, 0.0);
				setPose(q, p, g2g_next);
				transformPose(g2b_current, g2g_next, g2b_next);
				//=== According to model layer information to adjust the pz value of g2b_next. ===//
				g2b_next.position.z = object2base_pose.position.z + height[i];

				g2b_current = g2b_next;

				//For acquiring tactile feature
				if(( i == (height.size() -1)) && (j == 3))
					flag = true;
				if(flag == true)
				{
					flag = false;
					for (int k = 0; k < 6; k ++)
					{
						gripper2base_pose.push_back(g2b_next);
						enable_gripper_control_flag.push_back(true);
						enable_clamping_mode.push_back(true);
					}
				}

				gripper2base_pose.push_back(g2b_next);
				enable_gripper_control_flag.push_back(true);
				enable_clamping_mode.push_back(false);
			}
		}

		gripper2base_pose.push_back(g2b_initial_b);
		enable_gripper_control_flag.push_back(false);
		enable_clamping_mode.push_back(false);

		gripper2base_pose.push_back(g2b_home);
		enable_gripper_control_flag.push_back(false);
		enable_clamping_mode.push_back(false);
	}
}

void CentralController::transformPose(const geometry_msgs::Pose& old2base_pose,
		                              const geometry_msgs::Pose& new2old_pose,
										    geometry_msgs::Pose& new2base_pose)
{
	Eigen::Affine3d old2base_transform (Eigen::Affine3d::Identity ());
	Eigen::Affine3d new2old_transform  (Eigen::Affine3d::Identity ());
	Eigen::Affine3d new2base_transform (Eigen::Affine3d::Identity ());

	tf::poseMsgToEigen(old2base_pose, old2base_transform);
	tf::poseMsgToEigen(new2old_pose, new2old_transform);

	new2base_transform = old2base_transform * new2old_transform;

	tf::poseEigenToMsg(new2base_transform, new2base_pose);
}

bool CentralController::boolArmReachDesiredPose(const geometry_msgs::Pose& gripper2base_pose,
			                                    const double& position_threshold)
{
	double max_abs_error;
	try
	{
		tf::StampedTransform transform;
		listener.waitForTransform("base", "sdh_hand_gripper", ros::Time(0), ros::Duration(5));
		listener.lookupTransform("base", "sdh_hand_gripper", ros::Time(0), transform);

		double abs_position_x_error = fabs(gripper2base_pose.position.x - transform.getOrigin().getX());
		double abs_position_y_error = fabs(gripper2base_pose.position.y - transform.getOrigin().getY());
		double abs_position_z_error = fabs(gripper2base_pose.position.z - transform.getOrigin().getZ());

		if (abs_position_x_error > abs_position_y_error) max_abs_error = abs_position_x_error;
		else                                             max_abs_error = abs_position_y_error;
		if (max_abs_error < abs_position_z_error)        max_abs_error = abs_position_z_error;
		ROS_INFO_STREAM_THROTTLE(2, "max_abs_error: " << max_abs_error);
	}
	catch(tf::LookupException& ex)
	{
		ROS_ERROR("Received an exception trying to lookupTransform from \"sdh_hand_gripper\" to \"base\": %s", ex.what());
	}

	if (max_abs_error < position_threshold)  return true;
	else                                     return false;
}

bool CentralController::boolAllFingerContactOccurOrReachEndPose(const identification_msgs::PlannerStates&          planner_states,
                                                                const identification_msgs::TactileProcessorStates& tactile_processor_states)
{
	bool flag[3], all_flag;

	for (int i=0; i<3; i++)
		flag[i] = planner_states.finger_contact_failure_flag[i] | tactile_processor_states.fingertip_contact_pose_valid_flag[i];

	all_flag = flag[0] & flag[1] & flag[2];

	if (all_flag == true)  return true;
	else                   return false;
}

void CentralController::generateCommands(const identification_msgs::PlannerStates&                   planner_states,
						                 const identification_msgs::TactileProcessorStates&          tactile_processor_states,
										 const identification_msgs::IdentifierStates&                identifier_states,
										       identification_msgs::PlannerControlCommands&          planner_control_commands,
										       identification_msgs::TactileProcessorControlCommands& tactile_processor_control_commands,
										       identification_msgs::IdentifierControlCommands&       identifier_control_commands)
{
	//------------------------------------------------------------------------------------------//
	//------------------------------Start recognition process...--------------------------------//
	//------------------------------------------------------------------------------------------//
	static bool start_recognition_command = true;
	identifier_control_commands.start_recognition_command = start_recognition_command;
	identifier_control_commands.grasp_process_command     = false;
	identifier_control_commands.desired_object_name       = desired_object;
	identifier_control_commands.experiment_label          = experiment_label;
	identifier_control_command_pub.publish(identifier_control_commands);

	if (start_recognition_command == true)
	{
		start_recognition_command = false;

		//=== Wait until the num value is true. ===//
		ROS_WARN_STREAM("Wait until the num value is true..."); //for debug
		while (identifier_states.num_potential_target_valid_flag == false) usleep(10000);//0.01s

		ROS_WARN_STREAM("Number of potential targets :"<<identifier_states.num_potential_target); //for debug
		for (int i=0; i<identifier_states.num_potential_target; i++)
		{
			ROS_WARN_STREAM("Recognize the potential target_"<<i); //for debug
			//=== Wait until the pose value is true and check recognition result ===//
			ROS_INFO_STREAM("Wait until the pose value is true and check recognition result..."); //for debug
			while (identifier_states.potential_target_cluster_pose_valid_flag == false)
			{
				if (identifier_states.recognition_finished_flag == true) break;
				usleep(10000);//0.01s
			}
			if (identifier_states.recognition_finished_flag == true)
			{
				ROS_INFO_STREAM("Recognition is finished. recognition_success_flag:" << identifier_states.recognition_success_flag);
				break;
			}

			//=== Get potential target pose. ===//
			geometry_msgs::Pose object2base_pose = identifier_states.potential_target_cluster_pose_in_arm_base_frame;
			std::vector<geometry_msgs::Pose> gripper2base_pose;
			std::vector<bool>                enable_gripper_control_flag;
			std::vector<bool>                enable_clamping_mode;

			getGripperToBasePose(object2base_pose, gripper2base_pose, enable_gripper_control_flag, enable_clamping_mode);

			for (int index_pose=0; index_pose<gripper2base_pose.size(); index_pose++)
			{
				ROS_WARN_STREAM("Amount of poses: " << gripper2base_pose.size()<<"; index_pose: "<<(index_pose + 1)); //for debug

				if (index_pose == 0)
				{
					ROS_INFO_STREAM("Open the hand and reset tactile processor.");//for debug

					planner_control_commands.hand_grasp_command    = false;
				    planner_control_commands.hand_release_command  = true;
					planner_control_commands.clamping_mode_command = false;

					tactile_processor_control_commands.reset_tactile_processor_command = true;
					tactile_processor_control_command_pub.publish(tactile_processor_control_commands);

					identifier_control_commands.grasp_process_command  = true;
					identifier_control_command_pub.publish(identifier_control_commands);
				}
				else if (index_pose == (gripper2base_pose.size() - 1))
				{
					identifier_control_commands.grasp_process_command = false;
					identifier_control_command_pub.publish(identifier_control_commands);
				}

				planner_control_commands.gripper2base_pose = gripper2base_pose[index_pose];
				planner_control_commands.new_pose_command = true;
				planner_control_command_pub.publish(planner_control_commands);

				sleep(1);

				//=== Wait until the gripper reaches at the desired pose. ===//
				ROS_INFO_STREAM("Wait until the gripper reaches at the desired pose..."); //for debug
				bool   flag = false;
				double postion_threshold = 0.002; //unit: m
				int    counter = 0;
				while (flag == false)
				{
					flag = boolArmReachDesiredPose(gripper2base_pose[index_pose], postion_threshold);
					usleep(10000);//0.01s
					counter ++;
					if (counter > 500)	break;
				}

				if (enable_gripper_control_flag[index_pose] == true)
				{
					//=== Close the hand. ===//
					ROS_INFO_STREAM("Close the hand...");//for debug
					planner_control_commands.hand_grasp_command    = true;
					planner_control_commands.hand_release_command  = false;
					planner_control_commands.clamping_mode_command = enable_clamping_mode[index_pose];
					planner_control_commands.new_pose_command      = false;
					planner_control_command_pub.publish(planner_control_commands);

					tactile_processor_control_commands.reset_tactile_processor_command = false;
					tactile_processor_control_command_pub.publish(tactile_processor_control_commands);

					//=== Waiting under different conditions. ===//
					usleep(100000);//Delay 0.1s before checking
					if (planner_control_commands.clamping_mode_command == true)
					{
						//=== Wait until contact feature is valid or thumb contact failure is true... ===//
						while ((tactile_processor_states.contact_feature_valid_flag == false) &&
							   (planner_states.finger_contact_failure_flag[0] == false))
							usleep(10000);//0.01s
						sleep(1);//1s
					}
					else
					{
						//=== Wait until each finger contact occurs or reaches at the end pose... ===//
						flag = false;
						while (flag == false)
						{
							flag = boolAllFingerContactOccurOrReachEndPose(planner_states, tactile_processor_states);
							usleep(10000);//0.01s
						}
					}

					//=== Open the hand. ===//
					ROS_INFO_STREAM("Open the hand...");//for debug
					planner_control_commands.hand_grasp_command   = false;
					planner_control_commands.hand_release_command = true;
					planner_control_commands.new_pose_command     = false;
					planner_control_command_pub.publish(planner_control_commands);

					tactile_processor_control_commands.reset_tactile_processor_command = true;
					tactile_processor_control_command_pub.publish(tactile_processor_control_commands);

					//=== Waiting until open is successful. ===//
					while (planner_states.hand_open_success_flag == false)	usleep(10000);//0.01s
					usleep(2000000);//2s
				}
			}
		}

		//=== Waiting until the recognition is finished. ===//
		while (identifier_states.recognition_finished_flag == false)  usleep(10000);//0.01s

		if (identifier_states.num_potential_target == 0)
		{
			if (num_loop < 3)
			{
				start_recognition_command = true;
				identifier_control_commands.start_recognition_command = false;
				identifier_control_command_pub.publish(identifier_control_commands);
				usleep(3000000);//3s
			}
		}

		if (identifier_states.recognition_success_flag == true)
			ROS_WARN_STREAM("Recognition_loop_"<<num_loop<<": success.");
		else
			ROS_WARN_STREAM("Recognition_loop_"<<num_loop<<": failure.");

		num_loop ++;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "planner");

	CentralController central_controller_;

	ros::AsyncSpinner sp(4);
	sp.start();
	ros::waitForShutdown();

	return 0;
}

