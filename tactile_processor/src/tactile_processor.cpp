/*
 * tactile_processor_node.cpp
 *
 *  Created on: 17 Apr 2017
 *      Author: yuanfei
 */

#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <identification_msgs/TactileProcessorControlCommands.h>
#include <identification_msgs/TactileProcessorStates.h>
#include <kdl/frames.hpp>

class TactileProcessorClass
{
public:
	TactileProcessorClass();
private:
	double contact_event_force_threshold;
	double max_contact_force_threshold;
	double max_compression_threshold;

    geometry_msgs::WrenchStamped tactileForceOffset[3];

	identification_msgs::TactileProcessorControlCommands tactile_processor_control_commands;
	identification_msgs::TactileProcessorStates          tactile_processor_states;

	ros::Subscriber thumb_tactile_force_sub;
	ros::Subscriber finger1_tactile_force_sub;
	ros::Subscriber finger2_tactile_force_sub;
	ros::Subscriber tactile_processor_control_command_sub;

	ros::Publisher  tactile_processor_states_pub;

	ros::Timer timer;

	tf::TransformBroadcaster tf_broadcaster[3];
	tf::TransformListener listener;
    tf::Transform transform[3];

	void thumbTactileForceCallback(const geometry_msgs::WrenchStamped& msg);
	void finger1TactileForceCallback(const geometry_msgs::WrenchStamped& msg);
	void finger2TactileForceCallback(const geometry_msgs::WrenchStamped& msg);
	void timerEventCallback(const ros::TimerEvent& event);
	void tactileProcessorControlCommandCallback(const identification_msgs::TactileProcessorControlCommands& msg);
};

TactileProcessorClass::TactileProcessorClass()
{
	ros::NodeHandle nh("~");
    //-------------------------------------------------//
	nh.param("contact_event_force_threshold", contact_event_force_threshold, 0.21); //30*0.007 N
	nh.param("max_contact_force_threshold", max_contact_force_threshold, 10.0); //unit: N
	nh.param("max_compression_threshold", max_compression_threshold, 0.005); //unit: m
	//-------------------------------------------------//

    tactile_processor_states.fingertip_contact_pose_valid_flag = {false, false, false};
    tactile_processor_states.current_tactile_force = {0.0, 0.0, 0.0};

	tactile_processor_states.fingertip_contact_pose.resize(3);
	tactile_processor_states.contact_feature_valid_flag = false;

	thumb_tactile_force_sub   = nh.subscribe("/thumb_tactile_force", 1, &TactileProcessorClass::thumbTactileForceCallback, this);
	finger1_tactile_force_sub = nh.subscribe("/finger1_tactile_force", 1, &TactileProcessorClass::finger1TactileForceCallback, this);
	finger2_tactile_force_sub = nh.subscribe("/finger2_tactile_force", 1, &TactileProcessorClass::finger2TactileForceCallback, this);
	tactile_processor_control_command_sub = nh.subscribe("/tactile_processor_control_commands", 1, &TactileProcessorClass::tactileProcessorControlCommandCallback, this);

	tactile_processor_states_pub  = nh.advertise<identification_msgs::TactileProcessorStates>("/tactile_processor_states", 1, true);

	timer = nh.createTimer(ros::Duration(0.01), &TactileProcessorClass::timerEventCallback, this);

//	ROS_WARN_STREAM("zyf_test: " << 1); //for debug
}

void TactileProcessorClass::thumbTactileForceCallback(const geometry_msgs::WrenchStamped& msg)
{
	static bool   initial_distance_record_flag = false;
	static double initial_distance = 0.0;
	double fx, fy, fz;
	fx = msg.wrench.force.x - tactileForceOffset[0].wrench.force.x;
	fy = msg.wrench.force.y - tactileForceOffset[0].wrench.force.y;
	fz = msg.wrench.force.z - tactileForceOffset[0].wrench.force.z;

	tactile_processor_states.current_tactile_force[0] = sqrt(fx*fx + fy*fy + fz*fz);
	//-----reset force offset and contact pose& feature valid flag----//
	if (tactile_processor_control_commands.reset_tactile_processor_command == true)
	{
		tactileForceOffset[0] = msg;
		tactile_processor_states.fingertip_contact_pose_valid_flag[0] = false;
		tactile_processor_states.contact_feature_valid_flag = false;
		initial_distance_record_flag = false;
	}
	else
	{
		//-------calculate contact pose in finger-tip frame-----------//
		if (tactile_processor_states.current_tactile_force[0] > contact_event_force_threshold)
		{
//			ROS_INFO_STREAM("calculate contact pose and send transform ");
			double r     = 0.015;//unit:m
			double phi   = atan2(-fy, -fx);//unit: radian
			double alpha = 0.5*atan2(sqrt(fx*fx + fy*fy), -fz);//unit: radian

//			ROS_INFO_STREAM("thumb_alpha "<<alpha<<" thumb_phi "<<phi);
//			ROS_INFO_STREAM("thumb_serialNumber(MSE0A054) "<<msg.header.frame_id);

			geometry_msgs::Point position;
			position.x = r*cos(alpha);
			position.y = r*sin(alpha)*cos(phi);
			position.z = r*sin(alpha)*sin(phi);

			geometry_msgs::Quaternion q;
			KDL::Rotation rotation = rotation.RotX(phi)*rotation.RotZ(alpha);
			rotation.GetQuaternion(q.x, q.y, q.z, q.w);

			tactile_processor_states.fingertip_contact_pose[0].position = position;
			tactile_processor_states.fingertip_contact_pose[0].orientation = q;
			tactile_processor_states.fingertip_contact_pose_valid_flag[0] = true;

			//---------------for showing the contact frame in rivz-----------------//
			transform[0].setOrigin(tf::Vector3(position.x, position.y, position.z));
			transform[0].setRotation(tf::Quaternion(q.x, q.y, q.z, q.w));
			tf_broadcaster[0].sendTransform(tf::StampedTransform(transform[0], ros::Time::now(), "sdh_thumb_tactile_link", "sdh_thumb_tactile_contact_frame"));
		}
		else
		{
			tactile_processor_states.fingertip_contact_pose_valid_flag[0] = false;
		}

		//---------------------calculate contact feature ------------------------//
		if ((tactile_processor_states.fingertip_contact_pose_valid_flag[0] == true) &&
			(tactile_processor_states.fingertip_contact_pose_valid_flag[1] == true)	&&
			(tactile_processor_states.fingertip_contact_pose_valid_flag[2] == true))
		{

			if (initial_distance_record_flag == false)
			{
				initial_distance_record_flag = true;
				try
				{
					tf::StampedTransform transform;
					listener.lookupTransform("sdh_finger1_tactile_link", "sdh_thumb_tactile_link", ros::Time(0), transform);
					initial_distance = transform.getOrigin().length();
				}
				catch(tf::LookupException& ex)
				{
					ROS_ERROR("Received an exception trying to lookupTransform from \"sdh_thumb_tactile_link\" to \"sdh_finger1_tactile_link\": %s", ex.what());
				}
			}
			else
			{
				try
				{
					tf::StampedTransform transform;
					listener.lookupTransform("sdh_finger1_tactile_link", "sdh_thumb_tactile_link", ros::Time(0), transform);
					double s2 = transform.getOrigin().length();
					double s1 = initial_distance;
					double s0 = 0.066;//unit:m
					double delta_compression_distance = sqrt(s1*s1 + s0*s0/4.0) - sqrt(s2*s2 + s0*s0/4.0);

					if((tactile_processor_states.current_tactile_force[0] >= max_contact_force_threshold) ||
					   (delta_compression_distance >= max_compression_threshold))
					{
						if (delta_compression_distance > 0.0001)
						{
							tactile_processor_states.contact_feature_valid_flag = true;
							tactile_processor_states.contact_feature = tactile_processor_states.current_tactile_force[0]/delta_compression_distance;//unit:N/m
	//						ROS_INFO_STREAM("tactile feature: "<<tactile_processor_states.contact_feature);
						}
					}
				}
				catch(tf::LookupException& ex)
				{
					ROS_ERROR("Received an exception trying to lookupTransform from \"sdh_thumb_tactile_link\" to \"sdh_finger1_tactile_link\": %s", ex.what());
				}
			}
		}
		else
		{
			initial_distance_record_flag = false;
			tactile_processor_states.contact_feature_valid_flag = false;
		}
	}
}

void TactileProcessorClass::finger1TactileForceCallback(const geometry_msgs::WrenchStamped& msg)
{
	double fx, fy, fz;
	fx = msg.wrench.force.x - tactileForceOffset[1].wrench.force.x;
	fy = msg.wrench.force.y - tactileForceOffset[1].wrench.force.y;
	fz = msg.wrench.force.z - tactileForceOffset[1].wrench.force.z;

	tactile_processor_states.current_tactile_force[1] = sqrt(fx*fx + fy*fy + fz*fz);
	//-------reset force offset and contact pose valid flag-----------//
	if (tactile_processor_control_commands.reset_tactile_processor_command == true)
	{
		tactileForceOffset[1] = msg;
		tactile_processor_states.fingertip_contact_pose_valid_flag[1] = false;
	}
	else
	{
		//-------calculate contact pose in finger-tip frame-----------//
		if (tactile_processor_states.current_tactile_force[1] > contact_event_force_threshold)
		{
//			ROS_INFO_STREAM("calculate contact pose and send transform ");
			double r     = 0.015;//unit:m
			double phi   = atan2(-fy, -fx);//unit: radian
			double alpha = 0.5*atan2(sqrt(fx*fx + fy*fy), -fz);//unit: radian

//			ROS_INFO_STREAM("finger1_alpha "<<alpha<<" finger1_phi "<<phi);
//			ROS_INFO_STREAM("finger1_serialNumber(MSE0A039) "<<msg.header.frame_id);

			geometry_msgs::Point position;
			position.x = r*cos(alpha);
			position.y = r*sin(alpha)*cos(phi);
			position.z = r*sin(alpha)*sin(phi);

			geometry_msgs::Quaternion q;
			KDL::Rotation rotation = rotation.RotX(phi)*rotation.RotZ(alpha);
			rotation.GetQuaternion(q.x, q.y, q.z, q.w);

			tactile_processor_states.fingertip_contact_pose[1].position = position;
			tactile_processor_states.fingertip_contact_pose[1].orientation = q;
			tactile_processor_states.fingertip_contact_pose_valid_flag[1] = true;

			//---------------for showing the contact frame in rivz-----------------//
			transform[1].setOrigin(tf::Vector3(position.x, position.y, position.z));
			transform[1].setRotation(tf::Quaternion(q.x, q.y, q.z, q.w));
			tf_broadcaster[1].sendTransform(tf::StampedTransform(transform[1], ros::Time::now(), "sdh_finger1_tactile_link", "sdh_finger1_tactile_contact_frame"));
		}
		else
		{
			tactile_processor_states.fingertip_contact_pose_valid_flag[1] = false;
		}
	}
}

void TactileProcessorClass::finger2TactileForceCallback(const geometry_msgs::WrenchStamped& msg)
{
	double fx, fy, fz;
	fx = msg.wrench.force.x - tactileForceOffset[2].wrench.force.x;
	fy = msg.wrench.force.y - tactileForceOffset[2].wrench.force.y;
	fz = msg.wrench.force.z - tactileForceOffset[2].wrench.force.z;

	tactile_processor_states.current_tactile_force[2] = sqrt(fx*fx + fy*fy + fz*fz);
	//-------reset force offset and contact pose valid flag-----------//
	if (tactile_processor_control_commands.reset_tactile_processor_command == true)
	{
		tactileForceOffset[2] = msg;
		tactile_processor_states.fingertip_contact_pose_valid_flag[2] = false;
	}
	else
	{
		//-------calculate contact pose in finger-tip frame-----------//
		if (tactile_processor_states.current_tactile_force[2] > contact_event_force_threshold)
		{
//			ROS_INFO_STREAM("calculate contact pose and send transform ");
			double r     = 0.015;//unit:m
			double phi   = atan2(-fy, -fx);//unit: radian
			double alpha = 0.5*atan2(sqrt(fx*fx + fy*fy), -fz);//unit: radian

//			ROS_INFO_STREAM("finger2_alpha "<<alpha<<" finger2_phi "<<phi);
//			ROS_INFO_STREAM("finger2_serialNumber(MSE0A050) "<<msg.header.frame_id);

			geometry_msgs::Point position;
			position.x = r*cos(alpha);
			position.y = r*sin(alpha)*cos(phi);
			position.z = r*sin(alpha)*sin(phi);

			geometry_msgs::Quaternion q;
			KDL::Rotation rotation = rotation.RotX(phi)*rotation.RotZ(alpha);
			rotation.GetQuaternion(q.x, q.y, q.z, q.w);

			tactile_processor_states.fingertip_contact_pose[2].position = position;
			tactile_processor_states.fingertip_contact_pose[2].orientation = q;
			tactile_processor_states.fingertip_contact_pose_valid_flag[2] = true;

			//---------------for showing the contact frame in rivz-----------------//
			transform[2].setOrigin(tf::Vector3(position.x, position.y, position.z));
			transform[2].setRotation(tf::Quaternion(q.x, q.y, q.z, q.w));
			tf_broadcaster[2].sendTransform(tf::StampedTransform(transform[2], ros::Time::now(), "sdh_finger2_tactile_link", "sdh_finger2_tactile_contact_frame"));
		}
		else
		{
			tactile_processor_states.fingertip_contact_pose_valid_flag[2] = false;
		}
	}
}

void TactileProcessorClass::timerEventCallback(const ros::TimerEvent& event)
{
	tactile_processor_states_pub.publish(tactile_processor_states);
}

void TactileProcessorClass::tactileProcessorControlCommandCallback(const identification_msgs::TactileProcessorControlCommands& msg)
{
	tactile_processor_control_commands = msg;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "tactile_processor");

	TactileProcessorClass tactile_processor_class_;

//	ros::spin();

//	ros::MultiThreadedSpinner s(4);
//	ros::spin(s);

	ros::AsyncSpinner sp(4);
	sp.start();
	ros::waitForShutdown();

	return 0;
}
