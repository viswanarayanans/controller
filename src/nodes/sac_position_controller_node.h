

#ifndef RRC_CONTROL_SAC_POSITION_CONTROLLER_NODE_H
#define RRC_CONTROL_SAC_POSITION_CONTROLLER_NODE_H


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

#include "rrc_control/common.h"
#include "rrc_control/sac_position_controller.h"
#include "msg_check/PlotDataMsg.h"
#include "std_msgs/UInt8.h"
#include <serial_comm.h>

namespace rrc_control {
	class SacPositionControllerNode{
	public:
		SacPositionControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
		~SacPositionControllerNode();
		void InitializeParams();

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;

		SacPositionController position_controller_;
		msg_check::PlotDataMsg data_out_;
  		SerialComm comm_;

		ros::Subscriber cmd_mdj_traj_sub_;
		ros::Subscriber cmd_odom_sub_;
	  	ros::Subscriber pose_sub_;
	  	ros::Subscriber state_sub_;

		ros::Publisher motor_vel_pub_;
  		ros::Publisher plot_data_pub_;

		mav_msgs::EigenTrajectoryPointDeque commands_;
		std::deque<ros::Duration> command_waiting_times_;
		ros::Timer command_timer_;

  		EigenOdometry odometry;


		void MdjTrajCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);
		void TimedCallback(const ros::TimerEvent& e);
		void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
  		void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  		void StateCallback(const std_msgs::UInt8::ConstPtr& state_msg);
	};

}

#endif