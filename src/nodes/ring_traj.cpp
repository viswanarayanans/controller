#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>
#include <thread>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"

void pub_single_pt(ros::Publisher, double , double, double, double);
void start_trajectory(ros::Publisher);

int main(int argc, char **argv)

{

	ros::init(argc, argv, "traj_gen");
	ros::NodeHandle n;
	ros::Publisher trajectory_pub =
	      n.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
	          mav_msgs::default_topics::COMMAND_TRAJECTORY, 20);


	while (trajectory_pub.getNumSubscribers() == 0 && ros::ok()) {
		ROS_INFO("There is no subscriber available, trying again in 1 second.");
		ros::Duration(1.0).sleep();
	}
	ros::Duration(1.0).sleep();	

	pub_single_pt(trajectory_pub,1,-1,1,0);
	ros::Duration(5.0).sleep();
	start_trajectory(trajectory_pub);

	pub_single_pt(trajectory_pub,0.6,0,1,0);
	ros::Duration(5.0).sleep();
	pub_single_pt(trajectory_pub,0.6,0,0.32,0);
	ros::Duration(10.0).sleep();
	ROS_INFO_STREAM("Pick up the first payload");

	pub_single_pt(trajectory_pub,1,-1,1,0);
	ros::Duration(4.0).sleep();
	start_trajectory(trajectory_pub);
	ros::Duration(0.5).sleep();
	pub_single_pt(trajectory_pub,0.6,0,0.6,0);
	ROS_INFO_STREAM("Drop the first payload");
	ros::Duration(5.0).sleep();
	
	pub_single_pt(trajectory_pub,-0.6,0,1,0);
	ros::Duration(5.0).sleep();
	pub_single_pt(trajectory_pub,-0.6,0,0.32,0);
	ros::Duration(10.0).sleep();
	ROS_INFO_STREAM("Pick up the second payload");
	
	pub_single_pt(trajectory_pub,-0.6,0,1,0);
	ros::Duration(1.0).sleep();
	pub_single_pt(trajectory_pub,1,-1,1,0);
	ros::Duration(4.0).sleep();
	start_trajectory(trajectory_pub);
	ROS_INFO_STREAM("Drop the second payload");

	ros::spinOnce();
}

void pub_single_pt(ros::Publisher trajectory_pub, double x, double y, double z, double yaw){
	ros::Rate sleep_rate(1000);

	trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
	trajectory_msg.header.stamp = ros::Time::now();
	Eigen::Vector3d desired_position(x, y, z);
	double desired_yaw = yaw*M_PI / 180.0;

	mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
	  desired_position, desired_yaw, &trajectory_msg);

	ROS_INFO("Publishing waypoint: [%f, %f, %f].",
	       desired_position.x(),
	       desired_position.y(), desired_position.z());
	trajectory_pub.publish(trajectory_msg);
	sleep_rate.sleep();	
}

	/*
	while(ros::ok())
		{
	*/		
void start_trajectory(ros::Publisher trajectory_pub){
	// ros::Publisher trajectory_pub =
	//       n.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
	//           mav_msgs::default_topics::COMMAND_TRAJECTORY, 20);
	// ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_snap", 10);
	ros::Rate sleep_rate(100);
	int i = 0;
	mav_trajectory_generation::Vertex::Vector vertices;
	const int dimension = 3;
	const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
	mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

	start.makeStartOrEnd(Eigen::Vector3d(1, -1, 1), derivative_to_optimize);
	vertices.push_back(start);

	middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, -2, 1.0));
	vertices.push_back(middle);
	middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2, 0, 2.0));
	vertices.push_back(middle);
	middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, 2, 1.0));
	vertices.push_back(middle);
	middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(2, 0, 2.0));
	vertices.push_back(middle);
	middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1, -1, 1.5));
	vertices.push_back(middle);

	end.makeStartOrEnd(Eigen::Vector3d(0, 0, 1), derivative_to_optimize);
	vertices.push_back(end);

	std::vector<double> segment_times;
	const double v_max = 2;
	const double a_max = 2;
	segment_times = estimateSegmentTimes(vertices, v_max, a_max);

	const int N = 10;
	mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
	opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
	opt.solveLinear();

	mav_trajectory_generation::Segment::Vector segments;
	opt.getSegments(&segments);

	mav_trajectory_generation::Trajectory trajectory;
	opt.getTrajectory(&trajectory);

	// Single sample:
	double sampling_time = 2.0;
	int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
	Eigen::VectorXd sample = trajectory.evaluate(sampling_time, derivative_order);

	// Sample range:
	double t_start = 2.0;
	double t_end = 10.0;
	double dt = 0.01;
	std::vector<Eigen::VectorXd> result;
	std::vector<double> sampling_times; // Optional.
	trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);


	mav_msgs::EigenTrajectoryPoint state;
	mav_msgs::EigenTrajectoryPoint::Vector states;

	// Whole trajectory:
	double sampling_interval = 0.01;
	bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

	        
	int traj_size = states.size();        

	for(i=0; i<traj_size; i++)
	{
		trajectory_msgs::MultiDOFJointTrajectory traj_msg;
		mav_msgs::msgMultiDofJointTrajectoryFromEigen(states[i], &traj_msg);
		traj_msg.header.stamp = ros::Time::now();
		trajectory_pub.publish(traj_msg);
		sleep_rate.sleep();
	}
	//ROS_INFO("Published the point");

	visualization_msgs::MarkerArray markers;
	double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
	std::string frame_id = "world";

	// From Trajectory class:
	mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
	// vis_pub.publish(markers);
	vertices.clear();
	sleep_rate.sleep();
	ROS_INFO_STREAM("All states published");


	ros::Duration(1.0).sleep();
}
