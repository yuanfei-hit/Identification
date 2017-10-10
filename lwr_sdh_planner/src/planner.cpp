/********************************************************************************
 * planner.cpp
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
#include <sensor_msgs/JointState.h>//For sensor_msgs::JointState
#include <geometry_msgs/Pose.h>//For geometry_msgs::Pose
#include <exotica/Exotica.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <identification_msgs/PlannerControlCommands.h>
#include <identification_msgs/PlannerStates.h>
#include <identification_msgs/TactileProcessorStates.h>

using namespace exotica;


class PlannerClass
{
public:
	PlannerClass();
private:
	std::string lwr_chain_start;
	std::string lwr_chain_end;
	std::string urdf_param;
	std::string arm_online_test;
	std::string hand_online_test;

	double timeout;

	bool tactile_processor_state_valid_flag;
	bool planner_control_command_valid_flag;

	identification_msgs::PlannerStates                   planner_states;
	identification_msgs::TactileProcessorStates          tactile_processor_states;
	identification_msgs::PlannerControlCommands          planner_control_commands;

	sensor_msgs::JointState handInitialJoint;
	sensor_msgs::JointState armInitialJoint;
	sensor_msgs::JointState armDesireJoint;

	sensor_msgs::JointState armJointStates;
	sensor_msgs::JointState handJointStates;

	trajectory_msgs::JointTrajectory armJointTrajectory;
	trajectory_msgs::JointTrajectory handJointGraspTrajectory;
	moveit_msgs::DisplayTrajectory rvizDisplayTrajectory;

	ros::Subscriber arm_desire_pose_sub;
	ros::Subscriber joint_state_sub;
	ros::Subscriber planner_control_command_sub;
	ros::Subscriber tactile_processor_state_sub;

	ros::Publisher  joint_state_pub;
	ros::Publisher  arm_joint_trajectory_pub;
	ros::Publisher  hand_joint_trajectory_pub;
	ros::Publisher  rviz_display_trajectory_pub;
	ros::Publisher  planner_state_pub;

	ros::Timer timer0;
	ros::Timer timer1;

	void armDesirePoseCallback(const geometry_msgs::Pose& msg);
	void jointStatesCallback(const sensor_msgs::JointState& msg);
	void plannerControlCommandCallback(const identification_msgs::PlannerControlCommands& msg);
	void tactileProcessorStatesCallback(const identification_msgs::TactileProcessorStates& msg);
	void timer0EventCallback(const ros::TimerEvent& event);
	void timer1EventCallback(const ros::TimerEvent& event);

	void inverseKinematic(std::string chain_start,
                             std::string chain_end,
			                 std::string urdf_param,
			                 double timeout,
                             const sensor_msgs::JointState& joint_initial,
			                 const geometry_msgs::Pose& pose,
			                 sensor_msgs::JointState& joint_result);

	void armJointPlanner(const sensor_msgs::JointState& arm_current_joint,
			             const sensor_msgs::JointState& arm_target_joint,
						 trajectory_msgs::JointTrajectory& arm_joint_trajectory);

	void handJointPlanner(const sensor_msgs::JointState& hand_initial_joint,
						  trajectory_msgs::JointTrajectory& hand_joint_grasp_trajectory);
	void publishHandTrajectory(const identification_msgs::PlannerControlCommands&  planner_control_commands,
							   const identification_msgs::TactileProcessorStates&  tactile_processor_states,
							   const trajectory_msgs::JointTrajectory&             hand_joint_trajectory,
							         identification_msgs::PlannerStates&           planner_states);
};

PlannerClass::PlannerClass()
{
	ros::NodeHandle nh("~");

	nh.param("lwr_chain_start", lwr_chain_start, std::string(""));
	nh.param("lwr_chain_end", lwr_chain_end, std::string(""));

	if (lwr_chain_start=="" || lwr_chain_end=="")
	{
		ROS_FATAL("Missing chain info in launch file");
	  	exit (-1);
	}

	nh.param("timeout", timeout, 0.005);
	nh.param("arm_online_test", arm_online_test, std::string(""));
	nh.param("hand_online_test", hand_online_test, std::string(""));
	nh.param("urdf_param", urdf_param, std::string(""));

	tactile_processor_state_valid_flag = false;
	planner_control_command_valid_flag = false;

	planner_states.hand_grasp_failure_flag = false;
	planner_states.hand_open_success_flag  = false;
	planner_states.finger_contact_failure_flag = {false, false, false};

	double angle[2];
	angle[0] = -M_PI_2*2/3;
	angle[1] =  M_PI_2*1/3;
	handInitialJoint.name = {"sdh_knuckle_joint", "sdh_thumb_2_joint", "sdh_thumb_3_joint", "sdh_finger_12_joint", "sdh_finger_13_joint", "sdh_finger_22_joint", "sdh_finger_23_joint"};
	handInitialJoint.position = {0.0, angle[0], angle[1], angle[0], angle[1], angle[0], angle[1]};

	armInitialJoint.name = {"lwr_arm_0_joint", "lwr_arm_1_joint", "lwr_arm_2_joint", "lwr_arm_3_joint", "lwr_arm_4_joint", "lwr_arm_5_joint", "lwr_arm_6_joint"};
	armInitialJoint.position = {0.0, -0.05235092341899872, 0.0, 1.518426775932312, 0.0, -0.9599822759628296, 0.0};

	armJointStates  = armInitialJoint;
	armDesireJoint  = armInitialJoint;
	handJointStates = handInitialJoint;

	rvizDisplayTrajectory.trajectory.resize(2);

	arm_desire_pose_sub         = nh.subscribe("/arm_desire_pose", 1, &PlannerClass::armDesirePoseCallback, this);
	joint_state_sub             = nh.subscribe("/joint_states", 1, &PlannerClass::jointStatesCallback, this);
	planner_control_command_sub = nh.subscribe("/planner_control_commands", 1, &PlannerClass::plannerControlCommandCallback, this);
	tactile_processor_state_sub = nh.subscribe("/tactile_processor_states", 1, &PlannerClass::tactileProcessorStatesCallback, this);

	joint_state_pub             = nh.advertise<sensor_msgs::JointState>("/joint_states", 1, true);
	arm_joint_trajectory_pub    = nh.advertise<trajectory_msgs::JointTrajectory>("/kuka/robot_plan", 1, true);
	hand_joint_trajectory_pub   = nh.advertise<trajectory_msgs::JointTrajectory>("/hand_joint_trajectory", 1, true);
    rviz_display_trajectory_pub = nh.advertise<moveit_msgs::DisplayTrajectory>("/rviz_display_trajectory", 1, true);
    planner_state_pub           = nh.advertise<identification_msgs::PlannerStates>("/planner_states", 1, true);

	if (arm_online_test == "off" || hand_online_test == "off")
	{
		ROS_WARN_STREAM("Publish the topic of /joint_states without hardware.");
		timer0 = nh.createTimer(ros::Duration(0.01), &PlannerClass::timer0EventCallback, this);
	}
	timer1 = nh.createTimer(ros::Duration(0.033), &PlannerClass::timer1EventCallback, this);

	handJointPlanner(handInitialJoint, handJointGraspTrajectory);

//	ROS_WARN_STREAM("zyf_test: " << time); //for debug
}

void PlannerClass::armDesirePoseCallback(const geometry_msgs::Pose& msg)
{
	inverseKinematic(lwr_chain_start, lwr_chain_end, urdf_param, timeout, armJointStates, msg, armDesireJoint);
	armJointPlanner(armJointStates, armDesireJoint, armJointTrajectory);

	rvizDisplayTrajectory.trajectory[0].joint_trajectory = armJointTrajectory;
    rvizDisplayTrajectory.trajectory[1].joint_trajectory = handJointGraspTrajectory;

    arm_joint_trajectory_pub.publish(armJointTrajectory);       //Publish the trajectory to robot arm lwr
	rviz_display_trajectory_pub.publish(rvizDisplayTrajectory); //Publish the trajectory to rviz

	//=== For test the arm moving path by using rviz===//
	if (arm_online_test == "off")
		armJointStates.position = armDesireJoint.position;
}

void PlannerClass::jointStatesCallback(const sensor_msgs::JointState& msg)
{
	if (msg.name[0] == "lwr_arm_0_joint")   armJointStates  = msg;
	if (msg.name[0] == "sdh_knuckle_joint") handJointStates = msg;
}


void PlannerClass::plannerControlCommandCallback(const identification_msgs::PlannerControlCommands& msg)
{
	planner_control_commands = msg;
	planner_control_command_valid_flag = true;
	if (planner_control_commands.new_pose_command == true)
	{
		tf::Vector3  p;
		p.setValue(0.576113174177, -0.00999857647083, 0.494038047031);

		if ((p.getX() == planner_control_commands.gripper2base_pose.position.x) &&
		    (p.getY() == planner_control_commands.gripper2base_pose.position.y) &&
			(p.getZ() == planner_control_commands.gripper2base_pose.position.z))
			armDesireJoint = armInitialJoint;
		else
			inverseKinematic(lwr_chain_start, lwr_chain_end, urdf_param, timeout, armJointStates, planner_control_commands.gripper2base_pose, armDesireJoint);
		armJointPlanner(armJointStates, armDesireJoint, armJointTrajectory);

	    rvizDisplayTrajectory.trajectory[0].joint_trajectory = armJointTrajectory;
        rvizDisplayTrajectory.trajectory[1].joint_trajectory = handJointGraspTrajectory;

	    arm_joint_trajectory_pub.publish(armJointTrajectory);       //Publish the trajectory to robot arm lwr
	    rviz_display_trajectory_pub.publish(rvizDisplayTrajectory); //Publish the trajectory to rviz

	    //=== For test the arm moving path by using rviz===//
	    if (arm_online_test == "off")
	    	armJointStates.position = armDesireJoint.position;
	}
}

void PlannerClass::tactileProcessorStatesCallback(const identification_msgs::TactileProcessorStates& msg)
{
	tactile_processor_states = msg;
	tactile_processor_state_valid_flag = true;
}

void PlannerClass::timer0EventCallback(const ros::TimerEvent& event)
{
	if (arm_online_test == "off")
	{
		armJointStates.header.stamp = ros::Time::now();
		joint_state_pub.publish(armJointStates);
	}
	usleep(5000);//5ms
	if (hand_online_test == "off")
	{
		handJointStates.header.stamp = ros::Time::now();
		joint_state_pub.publish(handJointStates);
	}
}

void PlannerClass::timer1EventCallback(const ros::TimerEvent& event)
{
	publishHandTrajectory(planner_control_commands, tactile_processor_states, handJointGraspTrajectory, planner_states);
}

void PlannerClass::inverseKinematic(std::string chain_start,
		                          std::string chain_end,
								  std::string urdf_param,
								  double timeout,
		                          const sensor_msgs::JointState& joint_initial,
								  const geometry_msgs::Pose& pose,
								  sensor_msgs::JointState& joint_result)
{
	double eps = 1e-5;

	// This constructor parses the URDF loaded in rosparm urdf_param into the
	// needed KDL structures.
	// IK solver.
	TRAC_IK::TRAC_IK tracik_solver(chain_start, chain_end, urdf_param, timeout, eps);//, TRAC_IK::Manip2);

	KDL::Chain chain;
	KDL::JntArray ll, ul; //lower joint limits, upper joint limits

	bool valid = tracik_solver.getKDLChain(chain);

	if (!valid)
	{
		ROS_ERROR("There was no valid KDL chain found");
		return;
	}

	valid = tracik_solver.getKDLLimits(ll,ul);

	if (!valid)
	{
    	ROS_ERROR("There were no valid KDL joint limits found");
    	return;
	}

	assert(chain.getNrOfJoints() == ll.data.size());
	assert(chain.getNrOfJoints() == ul.data.size());

	ROS_INFO ("Using %d joints",chain.getNrOfJoints());

	// Set up KDL IK
	KDL::ChainFkSolverPos_recursive fk_solver(chain); // Forward kin. solver

	// Create Nominal chain configuration midway between all joint limits
	KDL::JntArray nominal(chain.getNrOfJoints());

	for (uint j=0; j<nominal.data.size(); j++)
	{
		nominal(j) = joint_initial.position[j];//(ll(j)+ul(j))/2.0;

	}


	boost::posix_time::ptime start_time;
	boost::posix_time::time_duration diff;

	KDL::JntArray result;
	KDL::Frame end_effector_pose;
	int rc;

	ROS_INFO_STREAM("*** Using TRAC-IK to calculate the values of CartToJnt");

	end_effector_pose.p.data[0] = pose.position.x;
	end_effector_pose.p.data[1] = pose.position.y;
	end_effector_pose.p.data[2] = pose.position.z;

	for (uint i=0; i<9; i++)  end_effector_pose.M.data[i] = end_effector_pose.M.Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w).data[i];


	double elapsed = 0;
	start_time = boost::posix_time::microsec_clock::local_time();
	rc         = tracik_solver.CartToJnt(nominal,end_effector_pose,result);
	diff       = boost::posix_time::microsec_clock::local_time() - start_time;
	elapsed    = diff.total_nanoseconds() / 1e9;

	if (rc>=0)  ROS_INFO_STREAM("TRAC-IK found a solution with "<<elapsed<<" secs");
	else   	    ROS_INFO_STREAM("TRAC-IK haven't found a solution");

	for (uint j=0; j<chain.getNrOfJoints(); j++)
	{
		ROS_INFO_STREAM("Joint "<<j<<" angle "<<result.data[j]);

		joint_result.position[j] = result.data[j];
	}
	joint_result.header.stamp = ros::Time::now();
}

void PlannerClass::armJointPlanner(const sensor_msgs::JointState& arm_current_joint,
                                   const sensor_msgs::JointState& arm_target_joint,
		                           trajectory_msgs::JointTrajectory& arm_joint_trajectory)
{
    ros::NodeHandle nh_("~");

    Initializer solver, problem;

    std::string file_name, solver_name, problem_name;
    nh_.getParam("ConfigurationFile",file_name);
    nh_.getParam("Solver",solver_name);
    nh_.getParam("Problem",problem_name);

    XMLLoader::load(file_name,solver, problem, solver_name, problem_name);

    HIGHLIGHT_NAMED("XMLnode","Loaded from XML");

    // Initialize

    PlanningProblem_ptr any_problem = Setup::createProblem(problem);
    MotionSolver_ptr any_solver = Setup::createSolver(solver);

    // Assign the problem to the solver
    any_solver->specifyProblem(any_problem);

    // If necessary, modify the problem after calling sol->specifyProblem()
    // e.g. set different rho:

    try
    {
        for (int t = 0; t < any_problem->getT(); t++)
        {
          // This sets the precision of all time steps BUT the last one to zero
          // This means we only aim to minimize the task cost in the last time step
          // The rest of the trajectory minimizes the control cost
          for(auto it : any_problem->getTaskDefinitions())
          {
              any_solver->setRho(it.first,0.0,t);
          }
        }
    }
    catch(Exception e) {}

    // Set goal state for bi-directional OMPL algorithms
    try
    {
        Eigen::VectorXd goal(7);
        for (uint i=0; i<7; i++) goal(i) = arm_target_joint.position[i];
        any_solver->setGoalState(goal);
    }
    catch(Exception e) {}

    // Create the initial configuration
    Eigen::VectorXd q(7);
    for (uint i=0; i<7; i++) q(i) = arm_current_joint.position[i];

    Eigen::MatrixXd solution;
    ROS_INFO_STREAM("Calling solve()");
    {
      ros::WallTime start_time = ros::WallTime::now();
      // Solve the problem using the AICO solver
      try
      {
        any_solver->Solve(q, solution);
        double time = ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();
        ROS_INFO_STREAM_THROTTLE(0.5, "Finished solving ("<<time<<"s)");
        ROS_INFO_STREAM_THROTTLE(0.5, "Solution "<<solution.row(solution.rows()-1));
        //-------------------------------------calculate duration------------------------------------------//
        double max_abs_delta_q, temp_abs_delta_q, time_interval;
        for (uint i=0; i<7; i++)
        {
        	temp_abs_delta_q = fabs(arm_target_joint.position[i] - arm_current_joint.position[i]);
        	if (max_abs_delta_q < temp_abs_delta_q) max_abs_delta_q = temp_abs_delta_q;
        }
        time_interval = max_abs_delta_q/solution.rows()/(20*3.14159265/180.0);//unit is second
        //-------------------------------------------------------------------------------------------------//
        arm_joint_trajectory.header.stamp = ros::Time::now();
        arm_joint_trajectory.header.seq++;
        arm_joint_trajectory.joint_names = any_problem->scene_->getSolver().getJointNames();

        arm_joint_trajectory.points.resize(solution.rows());

        for (uint i = 0; i < solution.rows(); i++)
        {
    		arm_joint_trajectory.points[i].positions.resize(solution.cols());

        	for (uint j = 0; j < solution.cols(); j++)
        	{
        		arm_joint_trajectory.points[i].positions[j] = solution(i, j);
        	}
        	arm_joint_trajectory.points[i].time_from_start = ros::Duration(time_interval * (i + 1));
        }
      }
      catch (SolveException e)
      {
        double time = ros::Duration((ros::WallTime::now() - start_time).toSec()).toSec();
        ROS_INFO_STREAM_THROTTLE(0.5, e.what()<<" ("<<time<<"s)");
      }
    }
}

void PlannerClass::handJointPlanner(const sensor_msgs::JointState& hand_initial_joint,
								    trajectory_msgs::JointTrajectory& hand_joint_grasp_trajectory)
{
	double l[2], delta_angle, init_angle[2], init_px, init_pz, end_px, interval_px, interval_time, angle[2], a, b, px, pz;
	int num_cartesian_space_plan, num_joint_space_plan;

	l[0]          = 0.0865; //unit meter
	l[1]          = 0.0620; //unit meter
	end_px        = 0.0;//0.01;   //unit meter

	interval_px   = 0.0005; //unit meter
	interval_time = 0.1;    //unit second

	delta_angle   = M_PI*68.23/180.0;

	init_angle[0] = -M_PI_4;
	init_angle[1] =  M_PI_4;

	init_px = l[0]*sin(init_angle[0]) + l[1]*cos(init_angle[0] + init_angle[1] - delta_angle);
	init_pz = l[0]*cos(init_angle[0]) - l[1]*sin(init_angle[0] + init_angle[1] - delta_angle);

	px = init_px;
	pz = init_pz;

	num_cartesian_space_plan = (end_px - init_px)/interval_px;

	//=================================================//
	double interval_angle = 0.5*M_PI/180.0;
	int num_temp[2];

	num_temp[0] = fabs(init_angle[0] - hand_initial_joint.position[1])/interval_angle;
	num_temp[1] = fabs(init_angle[1] - hand_initial_joint.position[2])/interval_angle;

	if (num_temp[0] > num_temp[1])	num_joint_space_plan = num_temp[0];
	else                     		num_joint_space_plan = num_temp[1];
	//=================================================//

	hand_joint_grasp_trajectory.header.stamp = ros::Time::now();
	hand_joint_grasp_trajectory.header.seq++;
	hand_joint_grasp_trajectory.joint_names = hand_initial_joint.name;
	hand_joint_grasp_trajectory.points.resize(num_cartesian_space_plan + num_joint_space_plan);

	for (int i = 0; i < (num_joint_space_plan + num_cartesian_space_plan); i++)
	{
		if (i < num_joint_space_plan)
		{
			angle[0] = hand_initial_joint.position[1] + (i + 1)*(init_angle[0] - hand_initial_joint.position[1])/num_joint_space_plan;
			angle[1] = hand_initial_joint.position[2] + (i + 1)*(init_angle[1] - hand_initial_joint.position[2])/num_joint_space_plan;
		}
		else
		{
			//--------------calculate angle[]------------------//
			px = init_px + interval_px * (i + 1 - num_joint_space_plan);
			angle[1] = asin((l[0]*l[0] + l[1]*l[1] - px*px - pz*pz)/(2.0*l[0]*l[1])) + delta_angle;

			a = l[0] - l[1]*sin(angle[1] - delta_angle);
			b = l[1]*cos(angle[1] - delta_angle);

			angle[0] = asin((a*px - b*pz)/(a*a + b*b));
			//-------------------------------------------------//
		}

		hand_joint_grasp_trajectory.points[i].positions.resize(7);

		hand_joint_grasp_trajectory.points[i].positions[0] = hand_initial_joint.position[0];
		hand_joint_grasp_trajectory.points[i].positions[1] = angle[0];
		hand_joint_grasp_trajectory.points[i].positions[2] = angle[1];
		hand_joint_grasp_trajectory.points[i].positions[3] = angle[0];
		hand_joint_grasp_trajectory.points[i].positions[4] = angle[1];
		hand_joint_grasp_trajectory.points[i].positions[5] = angle[0];
		hand_joint_grasp_trajectory.points[i].positions[6] = angle[1];

		hand_joint_grasp_trajectory.points[i].time_from_start = ros::Duration(interval_time * (i + 1));
	}
}

void PlannerClass::publishHandTrajectory(const identification_msgs::PlannerControlCommands&  planner_control_commands,
							             const identification_msgs::TactileProcessorStates&  tactile_processor_states,
							             const trajectory_msgs::JointTrajectory&             hand_joint_trajectory,
							                   identification_msgs::PlannerStates&           planner_states)
{
	//------------------------------------------------------------------------------------------//
	//------------------------------Publish hand joint trajectory-------------------------------//
	//------------------------------------------------------------------------------------------//
	if ((planner_control_command_valid_flag == true) &&
        (tactile_processor_state_valid_flag == true))
	{
		static int    point_index[3] = {0};
		static int    point_size     = hand_joint_trajectory.points.size() - 1;
		static double time_interval  = 0.5;// unit: second

		trajectory_msgs::JointTrajectory trajectory;
		trajectory.joint_names = handInitialJoint.name;
		trajectory.points.resize(1);
		trajectory.points[0].positions.resize(7);
		trajectory.points[0].velocities.resize(7);
		trajectory.points[0].time_from_start = ros::Duration(time_interval);

		//===Get desired hand joint position by looking up the table===//
		if ((planner_control_commands.hand_grasp_command   == true) &&
			(planner_control_commands.hand_release_command == false))
		{
			for (int i=0; i<3; i++)
				if (point_index[i] < point_size)  point_index[i]++;

			if ((tactile_processor_states.fingertip_contact_pose_valid_flag[0] &
				 tactile_processor_states.fingertip_contact_pose_valid_flag[1] &
				 tactile_processor_states.fingertip_contact_pose_valid_flag[2]) == false)
			{
				for (int i=0; i<3; i++)
					if (tactile_processor_states.fingertip_contact_pose_valid_flag[i] == true)  point_index[i]--;
			}
			else
			{
				if (planner_control_commands.clamping_mode_command == false)
					point_index[0]--;
				else if (tactile_processor_states.contact_feature_valid_flag == true)
				{
					point_index[0]--;
					ROS_INFO_STREAM_THROTTLE(2, "Grasp successes.");
				}
				point_index[1]--;
				point_index[2]--;
			}

			//=== Checking planner states ==//
			if ((point_index[0] == point_size) && (point_index[1] == point_size) && (point_index[2] == point_size))
			{
				ROS_INFO_STREAM_THROTTLE(2, "Grasp fails.");
				planner_states.hand_grasp_failure_flag = true;
			}
			else
			{
				planner_states.hand_open_success_flag  = false;
				planner_states.hand_grasp_failure_flag = false;

				for (int i=0; i<3; i++)
					planner_states.finger_contact_failure_flag[i] = false;
			}

			for (int i=0; i<3; i++)
				if (point_index[i] == point_size)  planner_states.finger_contact_failure_flag[i] = true;
		}
		else if ((planner_control_commands.hand_grasp_command   == false) &&
				 (planner_control_commands.hand_release_command == true))
		{
			for (int i=0; i<3; i++)
			{
				point_index[i] = point_index[i] - 5;
				if (point_index[i] < 0) point_index[i] = 0;
			}

			//=== Checking planner states ==//
			if ((point_index[0] == 0) && (point_index[1] == 0) && (point_index[2] == 0))
			{
				ROS_INFO_STREAM_THROTTLE(2, "Open successes.");
				planner_states.hand_open_success_flag = true;
			}
			else
			{
				planner_states.hand_open_success_flag  = false;
				planner_states.hand_grasp_failure_flag = false;
			}
			for (int i=0; i<3; i++)
				planner_states.finger_contact_failure_flag[i] = false;
		}
		//=== Prepare data for publish ===//
		trajectory.points[0].positions[0] = hand_joint_trajectory.points[point_index[0]].positions[0];
		trajectory.points[0].positions[1] = hand_joint_trajectory.points[point_index[0]].positions[1];
		trajectory.points[0].positions[2] = hand_joint_trajectory.points[point_index[0]].positions[2];
		trajectory.points[0].positions[3] = hand_joint_trajectory.points[point_index[1]].positions[3];
		trajectory.points[0].positions[4] = hand_joint_trajectory.points[point_index[1]].positions[4];
		trajectory.points[0].positions[5] = hand_joint_trajectory.points[point_index[2]].positions[5];
		trajectory.points[0].positions[6] = hand_joint_trajectory.points[point_index[2]].positions[6];

		for (int i=0; i<7; i++)
		{
			trajectory.points[0].velocities[i] = (trajectory.points[0].positions[i] - handJointStates.position[i])/time_interval;

			if      (trajectory.points[0].velocities[i] >  1.0) trajectory.points[0].velocities[i] =  1.0;
			else if (trajectory.points[0].velocities[i] < -1.0) trajectory.points[0].velocities[i] = -1.0;
		}

		if ((tactile_processor_states.fingertip_contact_pose_valid_flag[0] &
			 tactile_processor_states.fingertip_contact_pose_valid_flag[1] &
			 tactile_processor_states.fingertip_contact_pose_valid_flag[2]) == false)
		{
			if (tactile_processor_states.fingertip_contact_pose_valid_flag[0] == true)
				for (int i=0; i<3; i++)    trajectory.points[0].velocities[i] = 0.0;
			if (tactile_processor_states.fingertip_contact_pose_valid_flag[1] == true)
				for (int i=3; i<5; i++)    trajectory.points[0].velocities[i] = 0.0;
			if (tactile_processor_states.fingertip_contact_pose_valid_flag[2] == true)
				for (int i=5; i<7; i++)    trajectory.points[0].velocities[i] = 0.0;
		}
		else
		{
			if (planner_control_commands.clamping_mode_command == false)
				for (int i=0; i<7; i++)   trajectory.points[0].velocities[i] = 0.0;
			else
			{
				for (int i=3; i<7; i++)   trajectory.points[0].velocities[i] = 0.0;
				if (tactile_processor_states.contact_feature_valid_flag == true)
					for (int i=0; i<7; i++)   trajectory.points[0].velocities[i] = 0.0;
			}
		}

		trajectory.header.stamp = ros::Time::now();
	    hand_joint_trajectory_pub.publish(trajectory);
	    //=== For test the hand moving path by using rviz===//
	    if (hand_online_test == "off")
	    	for (int i=0;i<7;i++) handJointStates.position[i] = trajectory.points[0].positions[i];
	}
	//=== Publish the planner states. ===//
	planner_state_pub.publish(planner_states);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "planner");

	PlannerClass planner_class_;

//	ros::spin();

//	ros::MultiThreadedSpinner s(4);
//	ros::spin(s);

	ros::AsyncSpinner sp(4);
	sp.start();
	ros::waitForShutdown();

	return 0;
}
