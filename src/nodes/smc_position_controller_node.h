

#ifndef ASMC_CONTROL_SMC_POSITION_CONTROLLER_NODE_H
#define ASMC_CONTROL_SMC_POSITION_CONTROLLER_NODE_H


#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "viswa_control/common.h"
#include "viswa_control/smc_position_controller.h"

namespace viswa_control {
	class SmcPositionControllerNode{
	public:
		SmcPositionControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
		~SmcPositionControllerNode();
		void InitializeParams();

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;

		SmcPositionController position_controller_;

		ros::Subscriber cmd_mdj_traj_sub_;
		ros::Subscriber cmd_odom_sub_;
	  	ros::Subscriber pose_sub_;

		ros::Publisher motor_vel_pub_;

		mav_msgs::EigenTrajectoryPointDeque commands_;
		std::deque<ros::Duration> command_waiting_times_;
		ros::Timer command_timer_;

  		EigenOdometry odometry;


		void MdjTrajCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);
		void TimedCallback(const ros::TimerEvent& e);
		void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
  		void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
		
	};

}

#endif