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

int main(int argc, char **argv)

{

ros::init(argc, argv, "traj_gen");

ros::NodeHandle n;
ros::NodeHandle nh_private("~");
ros::Publisher trajectory_pub =
      n.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
// ROS_INFO_STREAM("Default")
ros::Publisher traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/pelican/command/trajectory", 20);
ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_snap", 100 );

int i = 0;
ros::Rate sleep_rate(100);

ros::Duration(12.0).sleep();

/*
while(ros::ok())
	{
*/		mav_trajectory_generation::Vertex::Vector vertices;
		const int dimension = 3;
		const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
		mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

		start.makeStartOrEnd(Eigen::Vector3d(-1, -1, 1), derivative_to_optimize);
		vertices.push_back(start);

		middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1, -7, 1.5));
		vertices.push_back(middle);


		end.makeStartOrEnd(Eigen::Vector3d(2, -8, 2), derivative_to_optimize);
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
			traj_pub.publish(traj_msg);
			sleep_rate.sleep();
		}
		//ROS_INFO("Published the point");

		visualization_msgs::MarkerArray markers;
		double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
		std::string frame_id = "world";

		// From Trajectory class:
		mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
		vis_pub.publish(markers);
		vertices.clear();
		sleep_rate.sleep();

		start.makeStartOrEnd(Eigen::Vector3d(2, -8, 2), derivative_to_optimize);
		vertices.push_back(start);

		middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(2, -8, 1));
		vertices.push_back(middle);

		end.makeStartOrEnd(Eigen::Vector3d(2, -8, 0.3), derivative_to_optimize);
		vertices.push_back(end);

		sampling_interval = 0.005;
		segment_times = estimateSegmentTimes(vertices, v_max, a_max);

		opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
		opt.solveLinear();
		opt.getSegments(&segments);
		opt.getTrajectory(&trajectory);
		sample = trajectory.evaluate(sampling_time, derivative_order);

		trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);
		success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

		        
		traj_size = states.size();        

		for(i=0; i<traj_size; i++)
		{
			trajectory_msgs::MultiDOFJointTrajectory traj_msg;
			mav_msgs::msgMultiDofJointTrajectoryFromEigen(states[i], &traj_msg);
			traj_msg.header.stamp = ros::Time::now();
			traj_pub.publish(traj_msg);
			sleep_rate.sleep();
		}
		// From Trajectory class:
		mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
		vis_pub.publish(markers);
		vertices.clear();
		sleep_rate.sleep();

		ros::Duration(3.0).sleep();

		Eigen::Vector3d desired_position(2.0, -8.0, 1.0);
		trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
		// mav_msgs::msgMultiDofJointTrajectoryFromEigen(desired_position, &trajectory_msg);	
		trajectory_msg.header.stamp = ros::Time::now();

		double desired_yaw = 0.0;

		// Overwrite defaults if set as node parameters.
		nh_private.param("x", desired_position.x(), desired_position.x());
		nh_private.param("y", desired_position.y(), desired_position.y());
		nh_private.param("z", desired_position.z(), desired_position.z());
		nh_private.param("yaw", desired_yaw, desired_yaw);

		mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
		  desired_position, desired_yaw, &trajectory_msg);
		trajectory_pub.publish(trajectory_msg);

		ros::Duration(3.0).sleep();


		start.makeStartOrEnd(Eigen::Vector3d(2, -8, 1), derivative_to_optimize);
		vertices.push_back(start);

		// middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(2, -8, 1));
		// vertices.push_back(middle);

		middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(6, -6, 2.8));
		vertices.push_back(middle);

		middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(9, -4, 2.5));
		vertices.push_back(middle);

		end.makeStartOrEnd(Eigen::Vector3d(10, -3, 2), derivative_to_optimize);
		vertices.push_back(end);

		sampling_interval = 0.01;
		segment_times = estimateSegmentTimes(vertices, v_max, a_max);

		opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
		opt.solveLinear();
		opt.getSegments(&segments);
		opt.getTrajectory(&trajectory);
		sample = trajectory.evaluate(sampling_time, derivative_order);

		trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);
		success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

		        
		traj_size = states.size();        

		for(i=0; i<traj_size; i++)
		{
			trajectory_msgs::MultiDOFJointTrajectory traj_msg;
			mav_msgs::msgMultiDofJointTrajectoryFromEigen(states[i], &traj_msg);
			traj_msg.header.stamp = ros::Time::now();
			traj_pub.publish(traj_msg);
			sleep_rate.sleep();
		}
		// From Trajectory class:
		mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
		vis_pub.publish(markers);
		vertices.clear();
		sleep_rate.sleep();


		start.makeStartOrEnd(Eigen::Vector3d(10, -3, 2), derivative_to_optimize);
		vertices.push_back(start);

		middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(10, -3, 1));
		vertices.push_back(middle);

		end.makeStartOrEnd(Eigen::Vector3d(10, -3, 0.6), derivative_to_optimize);
		vertices.push_back(end);

		segment_times = estimateSegmentTimes(vertices, v_max, a_max);

		opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
		opt.solveLinear();
		opt.getSegments(&segments);
		opt.getTrajectory(&trajectory);
		sample = trajectory.evaluate(sampling_time, derivative_order);

		trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);
		success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

		        
		traj_size = states.size();        

		for(i=0; i<traj_size; i++)
		{
			trajectory_msgs::MultiDOFJointTrajectory traj_msg;
			mav_msgs::msgMultiDofJointTrajectoryFromEigen(states[i], &traj_msg);
			traj_msg.header.stamp = ros::Time::now();
			traj_pub.publish(traj_msg);
			sleep_rate.sleep();
		}
		// From Trajectory class:
		mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
		vis_pub.publish(markers);
		vertices.clear();
		sleep_rate.sleep();


		ros::Duration(3.0).sleep();

		desired_position << 10.0, -3.0, 1.0;
		
		trajectory_msg.header.stamp = ros::Time::now();

		desired_yaw = 0.0;

		// Overwrite defaults if set as node parameters.
		nh_private.param("x", desired_position.x(), desired_position.x());
		nh_private.param("y", desired_position.y(), desired_position.y());
		nh_private.param("z", desired_position.z(), desired_position.z());
		nh_private.param("yaw", desired_yaw, desired_yaw);

		mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
		  desired_position, desired_yaw, &trajectory_msg);
		// trajectory_pub.publish(trajectory_msg);

		ros::Duration(3.0).sleep();


		start.makeStartOrEnd(Eigen::Vector3d(10, -3, 0.6), derivative_to_optimize);
		vertices.push_back(start);

		middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(10, -1.5, 1.5));
		vertices.push_back(middle);

		end.makeStartOrEnd(Eigen::Vector3d(10, 0, 2), derivative_to_optimize);
		vertices.push_back(end);

		sampling_interval = 0.01;
		segment_times = estimateSegmentTimes(vertices, v_max, a_max);

		opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
		opt.solveLinear();
		opt.getSegments(&segments);
		opt.getTrajectory(&trajectory);
		sample = trajectory.evaluate(sampling_time, derivative_order);

		trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);
		success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

		        
		traj_size = states.size();        

		for(i=0; i<traj_size; i++)
		{
			trajectory_msgs::MultiDOFJointTrajectory traj_msg;
			mav_msgs::msgMultiDofJointTrajectoryFromEigen(states[i], &traj_msg);
			traj_msg.header.stamp = ros::Time::now();
			traj_pub.publish(traj_msg);
			sleep_rate.sleep();
		}
		// From Trajectory class:
		mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
		vis_pub.publish(markers);
		vertices.clear();
		sleep_rate.sleep();


		start.makeStartOrEnd(Eigen::Vector3d(10, 0, 2), derivative_to_optimize);
		vertices.push_back(start);

		middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(10, 0, 1));
		vertices.push_back(middle);

		end.makeStartOrEnd(Eigen::Vector3d(10, 0, 0.3), derivative_to_optimize);
		vertices.push_back(end);

		sampling_interval = 0.005;
		segment_times = estimateSegmentTimes(vertices, v_max, a_max);

		opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
		opt.solveLinear();
		opt.getSegments(&segments);
		opt.getTrajectory(&trajectory);
		sample = trajectory.evaluate(sampling_time, derivative_order);

		trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);
		success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

		        
		traj_size = states.size();        

		for(i=0; i<traj_size; i++)
		{
			trajectory_msgs::MultiDOFJointTrajectory traj_msg;
			mav_msgs::msgMultiDofJointTrajectoryFromEigen(states[i], &traj_msg);
			traj_msg.header.stamp = ros::Time::now();
			traj_pub.publish(traj_msg);
			sleep_rate.sleep();
		}
		// From Trajectory class:
		mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
		vis_pub.publish(markers);
		vertices.clear();
		sleep_rate.sleep();


		ros::Duration(3.0).sleep();

		desired_position << 10.0, 0.0, 1.0;
		
		trajectory_msg.header.stamp = ros::Time::now();

		desired_yaw = 0.0;

		// Overwrite defaults if set as node parameters.
		nh_private.param("x", desired_position.x(), desired_position.x());
		nh_private.param("y", desired_position.y(), desired_position.y());
		nh_private.param("z", desired_position.z(), desired_position.z());
		nh_private.param("yaw", desired_yaw, desired_yaw);

		mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
		  desired_position, desired_yaw, &trajectory_msg);
		trajectory_pub.publish(trajectory_msg);

		ros::Duration(2.0).sleep();

		// start.makeStartOrEnd(Eigen::Vector3d(10, 0, 0.3), derivative_to_optimize);
		// vertices.push_back(start);

		start.makeStartOrEnd(Eigen::Vector3d(10, 0, 1.0), derivative_to_optimize);
		vertices.push_back(start);

		middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(9, 2, 2.3));
		vertices.push_back(middle);

		middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(5, 4, 2.3));
		vertices.push_back(middle);

		middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(3, 5, 2));
		vertices.push_back(middle);

		middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2, 2, 2));
		vertices.push_back(middle);

		middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, 0, 2));
		vertices.push_back(middle);

		middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, 0, 1));
		vertices.push_back(middle);

		end.makeStartOrEnd(Eigen::Vector3d(0, 0, 0.5), derivative_to_optimize);
		vertices.push_back(end);

		sampling_interval = 0.01;
		segment_times = estimateSegmentTimes(vertices, v_max, a_max);
		opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
		opt.solveLinear();
		opt.getSegments(&segments);
		opt.getTrajectory(&trajectory);
		sample = trajectory.evaluate(sampling_time, derivative_order);
		trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);
		success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);
		        
		traj_size = states.size();        

		for(i=0; i<traj_size; i++)
		{
			trajectory_msgs::MultiDOFJointTrajectory traj_msg;
			mav_msgs::msgMultiDofJointTrajectoryFromEigen(states[i], &traj_msg);
			traj_msg.header.stamp = ros::Time::now();
			traj_pub.publish(traj_msg);
			sleep_rate.sleep();
		}
		
		mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);


		vis_pub.publish(markers);
		vertices.clear();
		sleep_rate.sleep();


		

		// mav_trajectory_generation::Vertex::Vector vertices;
		// const int dimension = 3;
		// const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
		// mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

		// start.makeStartOrEnd(Eigen::Vector3d(-1, -1, 1), derivative_to_optimize);
		// vertices.push_back(start);

		// middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, -5, 1.5));
		// vertices.push_back(middle);

		// middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(2, -7, 2));
		// vertices.push_back(middle);

		// middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(6, -4, 3));
		// vertices.push_back(middle);

		// middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(9, -1, 2.5));
		// vertices.push_back(middle);

		// middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(10, 0, 2.1));
		// vertices.push_back(middle);

		// middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(10, 0, 1.5));
		// vertices.push_back(middle);

		// middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(10, 0, 1.1));
		// vertices.push_back(middle);

		// end.makeStartOrEnd(Eigen::Vector3d(10, 0, 0.6), derivative_to_optimize);
		// vertices.push_back(end);


		// std::vector<double> segment_times;
		// const double v_max = 2;
		// const double a_max = 2;
		// segment_times = estimateSegmentTimes(vertices, v_max, a_max);

		// const int N = 10;
		// mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
		// opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
		// opt.solveLinear();

		// mav_trajectory_generation::Segment::Vector segments;
		// opt.getSegments(&segments);

		// mav_trajectory_generation::Trajectory trajectory;
		// opt.getTrajectory(&trajectory);


		// // Splitting:
		// // mav_trajectory_generation::Trajectory x_trajectory = trajectory.getTrajectoryWithSingleDimension(1);

		// // Compositing:
		// // mav_trajectory_generation::Trajectory trajectory_with_yaw; trajectory.getTrajectoryWithAppendedDimension(yaw_trajectory, &trajectory_with_yaw);


		// // Single sample:
		// double sampling_time = 2.0;
		// int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
		// Eigen::VectorXd sample = trajectory.evaluate(sampling_time, derivative_order);

		// // Sample range:
		// double t_start = 2.0;
		// double t_end = 10.0;
		// double dt = 0.01;
		// std::vector<Eigen::VectorXd> result;
		// std::vector<double> sampling_times; // Optional.
		// trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);


		// mav_msgs::EigenTrajectoryPoint state;
		// mav_msgs::EigenTrajectoryPoint::Vector states;

		// // Whole trajectory:
		// double sampling_interval = 0.01;
		// bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

		        
		// int traj_size = states.size();        

		// for(i=0; i<traj_size; i++)
		// {
		// 	trajectory_msgs::MultiDOFJointTrajectory traj_msg;
		// 	mav_msgs::msgMultiDofJointTrajectoryFromEigen(states[i], &traj_msg);
		// 	traj_msg.header.stamp = ros::Time::now();

		
		// 	//ROS_INFO("Joint Trajectory Obtained");

		// 	        // publishing trajectory to send to non linear mpc controller

		// 	//std::cout << i <<": Trajectory: " << traj_msg.points[0].transforms[0].translation << std::endl;
		// 	traj_pub.publish(traj_msg);
		// 	sleep_rate.sleep();
		// }
		// //ROS_INFO("Published the point");

		// visualization_msgs::MarkerArray markers;
		// double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
		// std::string frame_id = "world";

		// // From Trajectory class:
		// mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);


		// vis_pub.publish(markers);
		// vertices.clear();
		// sleep_rate.sleep();

  // 		ros::Duration(2.0).sleep();

		// // Default desired position and yaw.
		// Eigen::Vector3d desired_position(10.0, 0.0, 0.3);
		// trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
		// // mav_msgs::msgMultiDofJointTrajectoryFromEigen(desired_position, &trajectory_msg);	
		// trajectory_msg.header.stamp = ros::Time::now();

		// double desired_yaw = 0.0;

		// // Overwrite defaults if set as node parameters.
		// nh_private.param("x", desired_position.x(), desired_position.x());
		// nh_private.param("y", desired_position.y(), desired_position.y());
		// nh_private.param("z", desired_position.z(), desired_position.z());
		// nh_private.param("yaw", desired_yaw, desired_yaw);

		// mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
		//   desired_position, desired_yaw, &trajectory_msg);

		// // ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
		// //        nh.getNamespace().c_str(), desired_position.x(),
		// //        desired_position.y(), desired_position.z());
		// trajectory_pub.publish(trajectory_msg);




  // 		ros::Duration(5.0).sleep();

  // 		desired_position << 10.0, 0.0, 1.0;
		// // mav_msgs::msgMultiDofJointTrajectoryFromEigen(desired_position, &trajectory_msg);
		// trajectory_msg.header.stamp = ros::Time::now();
		// nh_private.param("x", desired_position.x(), desired_position.x());
		// nh_private.param("y", desired_position.y(), desired_position.y());
		// nh_private.param("z", desired_position.z(), desired_position.z());
		// nh_private.param("yaw", desired_yaw, desired_yaw);

		// mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
		//   desired_position, desired_yaw, &trajectory_msg);
		// trajectory_pub.publish(trajectory_msg);  		

  // 		ros::Duration(2.0).sleep();



		// // mav_trajectory_generation::Vertex::Vector vertices;
		// // const int dimension = 3;
		// // const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
		// // mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

		// start.makeStartOrEnd(Eigen::Vector3d(10, 0, 1.0), derivative_to_optimize);
		// vertices.push_back(start);

		// start.makeStartOrEnd(Eigen::Vector3d(10, 0, 1.5), derivative_to_optimize);
		// vertices.push_back(start);

		// middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(9, 2, 2.5));
		// vertices.push_back(middle);

		// middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(5, 4, 2.5));
		// vertices.push_back(middle);

		// middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(3, 5, 2));
		// vertices.push_back(middle);

		// // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1, 5, 2));
		// // vertices.push_back(middle);

		// middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(-2, 2, 2));
		// vertices.push_back(middle);

		// middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, 0, 2));
		// vertices.push_back(middle);

		// middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(0, 0, 1));
		// vertices.push_back(middle);

		// // middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(10, 0, 1.1));
		// // vertices.push_back(middle);

		// end.makeStartOrEnd(Eigen::Vector3d(0, 0, 0.5), derivative_to_optimize);
		// vertices.push_back(end);


		// // std::vector<double> segment_times;
		// // const double v_max = 2;
		// // const double a_max = 2;
		// segment_times = estimateSegmentTimes(vertices, v_max, a_max);

		// // const int N = 10;
		// // mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
		// opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
		// opt.solveLinear();

		// // mav_trajectory_generation::Segment::Vector segments;
		// opt.getSegments(&segments);

		// // mav_trajectory_generation::Trajectory trajectory;
		// opt.getTrajectory(&trajectory);


		// // Splitting:
		// // mav_trajectory_generation::Trajectory x_trajectory = trajectory.getTrajectoryWithSingleDimension(1);

		// // Compositing:
		// // mav_trajectory_generation::Trajectory trajectory_with_yaw; trajectory.getTrajectoryWithAppendedDimension(yaw_trajectory, &trajectory_with_yaw);


		// // Single sample:
		// // double sampling_time = 2.0;
		// // int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
		// sample = trajectory.evaluate(sampling_time, derivative_order);

		// // Sample range:
		// // double t_start = 2.0;
		// // double t_end = 10.0;
		// // double dt = 0.01;
		// // std::vector<Eigen::VectorXd> result;
		// // std::vector<double> sampling_times; // Optional.
		// trajectory.evaluateRange(t_start, t_end, dt, derivative_order, &result, &sampling_times);


		// // mav_msgs::EigenTrajectoryPoint state;
		// // mav_msgs::EigenTrajectoryPoint::Vector states;

		// // Whole trajectory:
		// // double sampling_interval = 0.01;
		// success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);

		        
		// traj_size = states.size();        

		// for(i=0; i<traj_size; i++)
		// {
		// 	trajectory_msgs::MultiDOFJointTrajectory traj_msg;
		// 	mav_msgs::msgMultiDofJointTrajectoryFromEigen(states[i], &traj_msg);
		// 	traj_msg.header.stamp = ros::Time::now();
		
		// 	//ROS_INFO("Joint Trajectory Obtained");

		// 	        // publishing trajectory to send to non linear mpc controller

		// 	//std::cout << i <<": Trajectory: " << traj_msg.points[0].transforms[0].translation << std::endl;
		// 	traj_pub.publish(traj_msg);
		// 	sleep_rate.sleep();
		// }
		// //ROS_INFO("Published the point");

		// // visualization_msgs::MarkerArray markers;
		// // double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
		// // std::string frame_id = "world";

		// // From Trajectory class:
		// mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);


		// vis_pub.publish(markers);
		// vertices.clear();
		// sleep_rate.sleep();
		





















		ros::spinOnce();
	/*}*/


}