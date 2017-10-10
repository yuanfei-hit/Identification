/********************************************************************************
 * lwr_end_pos_pub_for_test.cpp
 *
 *  Max stable frequency almost equal to 200 Hz.
 *
 *  Created on: 30 Mar 2017
 *      Author: yuanfei
********************************************************************************/

#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <geometry_msgs/Pose.h>//For geometry_msgs::Pose

double fRand(double min, double max)
{
  srand(ros::Time::now().nsec);
  double f = (double)rand() / RAND_MAX;
  return min + f * (max - min);
}

void lwrKinematic(std::string chain_start, std::string chain_end, double timeout, std::string urdf_param, geometry_msgs::Pose &msg)
{

  double eps = 1e-5;

  // This constructor parses the URDF loaded in rosparm urdf_param into the
  // needed KDL structures.
  // IK solver.
  TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);

  KDL::Chain chain;
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);
  
  if (!valid) {
    ROS_ERROR("There was no valid KDL chain found");
    return;
  }

  valid = tracik_solver.getKDLLimits(ll,ul);

  if (!valid) {
    ROS_ERROR("There were no valid KDL joint limits found");
    return;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  ROS_INFO ("Using %d joints",chain.getNrOfJoints());

  KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver
  KDL::JntArray q(chain.getNrOfJoints());
  KDL::Frame end_effector_pose;

  double jointAngle1[7] = {-1.2696936130523682, -1.2441989183425903, 0.40441474318504333, 1.7034695148468018, -1.2133313417434692, 1.0999045372009277, 0.19446946680545807};
  double jointAngle2[7] = {-0.0715397372841835, -1.0819931030273438, 0.17556661367416382, 1.2563923597335815, 1.4395320415496826, 1.705618977546692, -0.7022716403007507};
  double jointAngle3[7] = {0.0, -0.05235092341899872, 0.0, 1.518426775932312, 0.0, -0.9599822759628296, 0.0};

  for (uint j=0; j<ll.data.size(); j++) {
   //  q(j)=fRand(ll(j), ul(j));
	  q(j) = jointAngle3[j];
  }

  ROS_INFO_STREAM("*** Calculate the value of end pose");

  fk_solver.JntToCart(q,end_effector_pose);

  msg.position.x = end_effector_pose.p.data[0];
  msg.position.y = end_effector_pose.p.data[1];
  msg.position.z = end_effector_pose.p.data[2];

  end_effector_pose.M.GetQuaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);

  ROS_INFO_STREAM("end_effector_pose: Vector p \n"
                  <<end_effector_pose.p.data[0]<<", "<<end_effector_pose.p.data[1]<<", "<<end_effector_pose.p.data[2]);
  ROS_INFO_STREAM("end_effector_pose: Quaternion q \n"
		          <<msg.orientation.x<<", "<<msg.orientation.y<<", "<<msg.orientation.z<<", "<<msg.orientation.w);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "lwr_end_pos_pub_for_test");
  ros::NodeHandle nh("~");

  std::string chain_start, chain_end, urdf_param;
  double timeout;

  nh.param("lwr_chain_start", chain_start, std::string(""));
  nh.param("lwr_chain_end", chain_end, std::string(""));

  if (chain_start=="" || chain_end=="") {
  	ROS_FATAL("Missing chain info in launch file");
  	exit (-1);
  }

  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));

  ros::Publisher pub = nh.advertise<geometry_msgs::Pose>("/arm_desire_pose", 1);
  ros::Duration(5).sleep();
  ros::Rate loop_rate(0.1);
  while(ros::ok()){
  //Create and fill in the message. The other four fields, which are ignored by robot, default to 0.
  geometry_msgs::Pose msg;

  lwrKinematic(chain_start, chain_end, timeout, urdf_param, msg);
  //Publish the message.
  pub.publish(msg);

  break; //publish only once
  loop_rate.sleep();
  }
  ros::waitForShutdown();
  return 0;
}
