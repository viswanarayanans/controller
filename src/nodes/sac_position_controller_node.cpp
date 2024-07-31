#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "sac_position_controller_node.h"

#include "rrc_control/parameters_ros.h"


namespace rrc_control {
	SacPositionControllerNode::SacPositionControllerNode(
		const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
	:nh_(nh), private_nh_(private_nh) {
		InitializeParams();

		cmd_mdj_traj_sub_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
			&SacPositionControllerNode::MdjTrajCallback, this);

		cmd_odom_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
			&SacPositionControllerNode::OdometryCallback, this);


		pose_sub_ = nh_.subscribe("/mavros/vision_pose/pose", 1, 
                            &SacPositionControllerNode::PoseCallback, this);


		state_sub_ = nh_.subscribe("/switch_state", 1, 
                            &SacPositionControllerNode::StateCallback, this);


		motor_vel_pub_ = nh_.advertise<mav_msgs::Actuators>(
			mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  		plot_data_pub_ = nh_.advertise<rrc_control::PlotDataMsg>("/data_out", 1);


		command_timer_ = nh_.createTimer(ros::Duration(0), &SacPositionControllerNode::TimedCallback, this,
			true, false);
	}

	SacPositionControllerNode::~SacPositionControllerNode() {}

	void SacPositionControllerNode::InitializeParams() {
		GetRosParameter(private_nh_, "theta_p/x",
                  position_controller_.controller_parameters_.theta_p_.x(),
                  &position_controller_.controller_parameters_.theta_p_.x());
		GetRosParameter(private_nh_, "theta_p/y",
                  position_controller_.controller_parameters_.theta_p_.y(),
                  &position_controller_.controller_parameters_.theta_p_.y());
		GetRosParameter(private_nh_, "theta_p/z",
                  position_controller_.controller_parameters_.theta_p_.z(),
                  &position_controller_.controller_parameters_.theta_p_.z());

		GetRosParameter(private_nh_, "theta_q/x",
                  position_controller_.controller_parameters_.theta_q_.x(),
                  &position_controller_.controller_parameters_.theta_q_.x());
		GetRosParameter(private_nh_, "theta_q/y",
                  position_controller_.controller_parameters_.theta_q_.y(),
                  &position_controller_.controller_parameters_.theta_q_.y());
		GetRosParameter(private_nh_, "theta_q/z",
                  position_controller_.controller_parameters_.theta_q_.z(),
                  &position_controller_.controller_parameters_.theta_q_.z());

		GetRosParameter(private_nh_, "alpha_p0_1/x",
                  position_controller_.controller_parameters_.alpha_p0_1_.x(),
                  &position_controller_.controller_parameters_.alpha_p0_1_.x());
		GetRosParameter(private_nh_, "alpha_p0_1/y",
                  position_controller_.controller_parameters_.alpha_p0_1_.y(),
                  &position_controller_.controller_parameters_.alpha_p0_1_.y());
		GetRosParameter(private_nh_, "alpha_p0_1/z",
                  position_controller_.controller_parameters_.alpha_p0_1_.z(),
                  &position_controller_.controller_parameters_.alpha_p0_1_.z());

		GetRosParameter(private_nh_, "alpha_p1_1/x",
                  position_controller_.controller_parameters_.alpha_p1_1_.x(),
                  &position_controller_.controller_parameters_.alpha_p1_1_.x());
		GetRosParameter(private_nh_, "alpha_p1_1/y",
                  position_controller_.controller_parameters_.alpha_p1_1_.y(),
                  &position_controller_.controller_parameters_.alpha_p1_1_.y());
		GetRosParameter(private_nh_, "alpha_p1_1/z",
                  position_controller_.controller_parameters_.alpha_p1_1_.z(),
                  &position_controller_.controller_parameters_.alpha_p1_1_.z());

		GetRosParameter(private_nh_, "alpha_q0_1/x",
                  position_controller_.controller_parameters_.alpha_q0_1_.x(),
                  &position_controller_.controller_parameters_.alpha_q0_1_.x());
		GetRosParameter(private_nh_, "alpha_q0_1/y",
                  position_controller_.controller_parameters_.alpha_q0_1_.y(),
                  &position_controller_.controller_parameters_.alpha_q0_1_.y());
		GetRosParameter(private_nh_, "alpha_q0_1/z",
                  position_controller_.controller_parameters_.alpha_q0_1_.z(),
                  &position_controller_.controller_parameters_.alpha_q0_1_.z());

		GetRosParameter(private_nh_, "alpha_q1_1/x",
                  position_controller_.controller_parameters_.alpha_q1_1_.x(),
                  &position_controller_.controller_parameters_.alpha_q1_1_.x());
		GetRosParameter(private_nh_, "alpha_q1_1/y",
                  position_controller_.controller_parameters_.alpha_q1_1_.y(),
                  &position_controller_.controller_parameters_.alpha_q1_1_.y());
		GetRosParameter(private_nh_, "alpha_q1_1/z",
                  position_controller_.controller_parameters_.alpha_q1_1_.z(),
                  &position_controller_.controller_parameters_.alpha_q1_1_.z());

		GetRosParameter(private_nh_, "alpha_q2_1/x",
                  position_controller_.controller_parameters_.alpha_q2_1_.x(),
                  &position_controller_.controller_parameters_.alpha_q2_1_.x());
		GetRosParameter(private_nh_, "alpha_q2_1/y",
                  position_controller_.controller_parameters_.alpha_q2_1_.y(),
                  &position_controller_.controller_parameters_.alpha_q2_1_.y());
		GetRosParameter(private_nh_, "alpha_q2_1/z",
                  position_controller_.controller_parameters_.alpha_q2_1_.z(),
                  &position_controller_.controller_parameters_.alpha_q2_1_.z());

            GetRosParameter(private_nh_, "alpha_p0_2/x",
                  position_controller_.controller_parameters_.alpha_p0_2_.x(),
                  &position_controller_.controller_parameters_.alpha_p0_2_.x());
            GetRosParameter(private_nh_, "alpha_p0_2/y",
                  position_controller_.controller_parameters_.alpha_p0_2_.y(),
                  &position_controller_.controller_parameters_.alpha_p0_2_.y());
            GetRosParameter(private_nh_, "alpha_p0_2/z",
                  position_controller_.controller_parameters_.alpha_p0_2_.z(),
                  &position_controller_.controller_parameters_.alpha_p0_2_.z());

            GetRosParameter(private_nh_, "alpha_p1_2/x",
                  position_controller_.controller_parameters_.alpha_p1_2_.x(),
                  &position_controller_.controller_parameters_.alpha_p1_2_.x());
            GetRosParameter(private_nh_, "alpha_p1_2/y",
                  position_controller_.controller_parameters_.alpha_p1_2_.y(),
                  &position_controller_.controller_parameters_.alpha_p1_2_.y());
            GetRosParameter(private_nh_, "alpha_p1_2/z",
                  position_controller_.controller_parameters_.alpha_p1_2_.z(),
                  &position_controller_.controller_parameters_.alpha_p1_2_.z());

            GetRosParameter(private_nh_, "alpha_q0_2/x",
                  position_controller_.controller_parameters_.alpha_q0_2_.x(),
                  &position_controller_.controller_parameters_.alpha_q0_2_.x());
            GetRosParameter(private_nh_, "alpha_q0_2/y",
                  position_controller_.controller_parameters_.alpha_q0_2_.y(),
                  &position_controller_.controller_parameters_.alpha_q0_2_.y());
            GetRosParameter(private_nh_, "alpha_q0_2/z",
                  position_controller_.controller_parameters_.alpha_q0_2_.z(),
                  &position_controller_.controller_parameters_.alpha_q0_2_.z());

            GetRosParameter(private_nh_, "alpha_q1_2/x",
                  position_controller_.controller_parameters_.alpha_q1_2_.x(),
                  &position_controller_.controller_parameters_.alpha_q1_2_.x());
            GetRosParameter(private_nh_, "alpha_q1_2/y",
                  position_controller_.controller_parameters_.alpha_q1_2_.y(),
                  &position_controller_.controller_parameters_.alpha_q1_2_.y());
            GetRosParameter(private_nh_, "alpha_q1_2/z",
                  position_controller_.controller_parameters_.alpha_q1_2_.z(),
                  &position_controller_.controller_parameters_.alpha_q1_2_.z());

            GetRosParameter(private_nh_, "alpha_q2_2/x",
                  position_controller_.controller_parameters_.alpha_q2_2_.x(),
                  &position_controller_.controller_parameters_.alpha_q2_2_.x());
            GetRosParameter(private_nh_, "alpha_q2_2/y",
                  position_controller_.controller_parameters_.alpha_q2_2_.y(),
                  &position_controller_.controller_parameters_.alpha_q2_2_.y());
            GetRosParameter(private_nh_, "alpha_q2_2/z",
                  position_controller_.controller_parameters_.alpha_q2_2_.z(),
                  &position_controller_.controller_parameters_.alpha_q2_2_.z());

            GetRosParameter(private_nh_, "alpha_p0_3/x",
                  position_controller_.controller_parameters_.alpha_p0_3_.x(),
                  &position_controller_.controller_parameters_.alpha_p0_3_.x());
            GetRosParameter(private_nh_, "alpha_p0_3/y",
                  position_controller_.controller_parameters_.alpha_p0_3_.y(),
                  &position_controller_.controller_parameters_.alpha_p0_3_.y());
            GetRosParameter(private_nh_, "alpha_p0_3/z",
                  position_controller_.controller_parameters_.alpha_p0_3_.z(),
                  &position_controller_.controller_parameters_.alpha_p0_3_.z());

            GetRosParameter(private_nh_, "alpha_p1_3/x",
                  position_controller_.controller_parameters_.alpha_p1_3_.x(),
                  &position_controller_.controller_parameters_.alpha_p1_3_.x());
            GetRosParameter(private_nh_, "alpha_p1_3/y",
                  position_controller_.controller_parameters_.alpha_p1_3_.y(),
                  &position_controller_.controller_parameters_.alpha_p1_3_.y());
            GetRosParameter(private_nh_, "alpha_p1_3/z",
                  position_controller_.controller_parameters_.alpha_p1_3_.z(),
                  &position_controller_.controller_parameters_.alpha_p1_3_.z());

            GetRosParameter(private_nh_, "alpha_q0_3/x",
                  position_controller_.controller_parameters_.alpha_q0_3_.x(),
                  &position_controller_.controller_parameters_.alpha_q0_3_.x());
            GetRosParameter(private_nh_, "alpha_q0_3/y",
                  position_controller_.controller_parameters_.alpha_q0_3_.y(),
                  &position_controller_.controller_parameters_.alpha_q0_3_.y());
            GetRosParameter(private_nh_, "alpha_q0_3/z",
                  position_controller_.controller_parameters_.alpha_q0_3_.z(),
                  &position_controller_.controller_parameters_.alpha_q0_3_.z());

            GetRosParameter(private_nh_, "alpha_q1_3/x",
                  position_controller_.controller_parameters_.alpha_q1_3_.x(),
                  &position_controller_.controller_parameters_.alpha_q1_3_.x());
            GetRosParameter(private_nh_, "alpha_q1_3/y",
                  position_controller_.controller_parameters_.alpha_q1_3_.y(),
                  &position_controller_.controller_parameters_.alpha_q1_3_.y());
            GetRosParameter(private_nh_, "alpha_q1_3/z",
                  position_controller_.controller_parameters_.alpha_q1_3_.z(),
                  &position_controller_.controller_parameters_.alpha_q1_3_.z());

            GetRosParameter(private_nh_, "alpha_q2_3/x",
                  position_controller_.controller_parameters_.alpha_q2_3_.x(),
                  &position_controller_.controller_parameters_.alpha_q2_3_.x());
            GetRosParameter(private_nh_, "alpha_q2_3/y",
                  position_controller_.controller_parameters_.alpha_q2_3_.y(),
                  &position_controller_.controller_parameters_.alpha_q2_3_.y());
            GetRosParameter(private_nh_, "alpha_q2_3/z",
                  position_controller_.controller_parameters_.alpha_q2_3_.z(),
                  &position_controller_.controller_parameters_.alpha_q2_3_.z());


		GetRosParameter(private_nh_, "hatKp0_1/x",
                  position_controller_.controller_parameters_.hatKp0_1_.x(),
                  &position_controller_.controller_parameters_.hatKp0_1_.x());
		GetRosParameter(private_nh_, "hatKp0_1/y",
                  position_controller_.controller_parameters_.hatKp0_1_.y(),
                  &position_controller_.controller_parameters_.hatKp0_1_.y());
		GetRosParameter(private_nh_, "hatKp0_1/z",
                  position_controller_.controller_parameters_.hatKp0_1_.z(),
                  &position_controller_.controller_parameters_.hatKp0_1_.z());

		GetRosParameter(private_nh_, "hatKp1_1/x",
                  position_controller_.controller_parameters_.hatKp1_1_.x(),
                  &position_controller_.controller_parameters_.hatKp1_1_.x());
		GetRosParameter(private_nh_, "hatKp1_1/y",
                  position_controller_.controller_parameters_.hatKp1_1_.y(),
                  &position_controller_.controller_parameters_.hatKp1_1_.y());
		GetRosParameter(private_nh_, "hatKp1_1/z",
                  position_controller_.controller_parameters_.hatKp1_1_.z(),
                  &position_controller_.controller_parameters_.hatKp1_1_.z());

		GetRosParameter(private_nh_, "hatKq0_1/x",
                  position_controller_.controller_parameters_.hatKq0_1_.x(),
                  &position_controller_.controller_parameters_.hatKq0_1_.x());
		GetRosParameter(private_nh_, "hatKq0_1/y",
                  position_controller_.controller_parameters_.hatKq0_1_.y(),
                  &position_controller_.controller_parameters_.hatKq0_1_.y());
		GetRosParameter(private_nh_, "hatKq0_1/z",
                  position_controller_.controller_parameters_.hatKq0_1_.z(),
                  &position_controller_.controller_parameters_.hatKq0_1_.z());

		GetRosParameter(private_nh_, "hatKq1_1/x",
                  position_controller_.controller_parameters_.hatKq1_1_.x(),
                  &position_controller_.controller_parameters_.hatKq1_1_.x());
		GetRosParameter(private_nh_, "hatKq1_1/y",
                  position_controller_.controller_parameters_.hatKq1_1_.y(),
                  &position_controller_.controller_parameters_.hatKq1_1_.y());
		GetRosParameter(private_nh_, "hatKq1_1/z",
                  position_controller_.controller_parameters_.hatKq1_1_.z(),
                  &position_controller_.controller_parameters_.hatKq1_1_.z());

		GetRosParameter(private_nh_, "hatKq2_1/x",
                  position_controller_.controller_parameters_.hatKq2_1_.x(),
                  &position_controller_.controller_parameters_.hatKq2_1_.x());
		GetRosParameter(private_nh_, "hatKq2_1/y",
                  position_controller_.controller_parameters_.hatKq2_1_.y(),
                  &position_controller_.controller_parameters_.hatKq2_1_.y());
		GetRosParameter(private_nh_, "hatKq2_1/z",
                  position_controller_.controller_parameters_.hatKq2_1_.z(),
                  &position_controller_.controller_parameters_.hatKq2_1_.z());

		GetRosParameter(private_nh_, "hatKp0_2/x",
                  position_controller_.controller_parameters_.hatKp0_2_.x(),
                  &position_controller_.controller_parameters_.hatKp0_2_.x());
		GetRosParameter(private_nh_, "hatKp0_2/y",
                  position_controller_.controller_parameters_.hatKp0_2_.y(),
                  &position_controller_.controller_parameters_.hatKp0_2_.y());
		GetRosParameter(private_nh_, "hatKp0_2/z",
                  position_controller_.controller_parameters_.hatKp0_2_.z(),
                  &position_controller_.controller_parameters_.hatKp0_2_.z());

		GetRosParameter(private_nh_, "hatKp1_2/x",
                  position_controller_.controller_parameters_.hatKp1_2_.x(),
                  &position_controller_.controller_parameters_.hatKp1_2_.x());
		GetRosParameter(private_nh_, "hatKp1_2/y",
                  position_controller_.controller_parameters_.hatKp1_2_.y(),
                  &position_controller_.controller_parameters_.hatKp1_2_.y());
		GetRosParameter(private_nh_, "hatKp1_2/z",
                  position_controller_.controller_parameters_.hatKp1_2_.z(),
                  &position_controller_.controller_parameters_.hatKp1_2_.z());

		GetRosParameter(private_nh_, "hatKq0_2/x",
                  position_controller_.controller_parameters_.hatKq0_2_.x(),
                  &position_controller_.controller_parameters_.hatKq0_2_.x());
		GetRosParameter(private_nh_, "hatKq0_2/y",
                  position_controller_.controller_parameters_.hatKq0_2_.y(),
                  &position_controller_.controller_parameters_.hatKq0_2_.y());
		GetRosParameter(private_nh_, "hatKq0_2/z",
                  position_controller_.controller_parameters_.hatKq0_2_.z(),
                  &position_controller_.controller_parameters_.hatKq0_2_.z());

		GetRosParameter(private_nh_, "hatKq1_2/x",
                  position_controller_.controller_parameters_.hatKq1_2_.x(),
                  &position_controller_.controller_parameters_.hatKq1_2_.x());
		GetRosParameter(private_nh_, "hatKq1_2/y",
                  position_controller_.controller_parameters_.hatKq1_2_.y(),
                  &position_controller_.controller_parameters_.hatKq1_2_.y());
		GetRosParameter(private_nh_, "hatKq1_2/z",
                  position_controller_.controller_parameters_.hatKq1_2_.z(),
                  &position_controller_.controller_parameters_.hatKq1_2_.z());

		GetRosParameter(private_nh_, "hatKq2_2/x",
                  position_controller_.controller_parameters_.hatKq2_2_.x(),
                  &position_controller_.controller_parameters_.hatKq2_2_.x());
		GetRosParameter(private_nh_, "hatKq2_2/y",
                  position_controller_.controller_parameters_.hatKq2_2_.y(),
                  &position_controller_.controller_parameters_.hatKq2_2_.y());
		GetRosParameter(private_nh_, "hatKq2_2/z",
                  position_controller_.controller_parameters_.hatKq2_2_.z(),
                  &position_controller_.controller_parameters_.hatKq2_2_.z());

            GetRosParameter(private_nh_, "hatKp0_3/x",
                  position_controller_.controller_parameters_.hatKp0_3_.x(),
                  &position_controller_.controller_parameters_.hatKp0_3_.x());
            GetRosParameter(private_nh_, "hatKp0_3/y",
                  position_controller_.controller_parameters_.hatKp0_3_.y(),
                  &position_controller_.controller_parameters_.hatKp0_3_.y());
            GetRosParameter(private_nh_, "hatKp0_3/z",
                  position_controller_.controller_parameters_.hatKp0_3_.z(),
                  &position_controller_.controller_parameters_.hatKp0_3_.z());

            GetRosParameter(private_nh_, "hatKp1_3/x",
                  position_controller_.controller_parameters_.hatKp1_3_.x(),
                  &position_controller_.controller_parameters_.hatKp1_3_.x());
            GetRosParameter(private_nh_, "hatKp1_3/y",
                  position_controller_.controller_parameters_.hatKp1_3_.y(),
                  &position_controller_.controller_parameters_.hatKp1_3_.y());
            GetRosParameter(private_nh_, "hatKp1_3/z",
                  position_controller_.controller_parameters_.hatKp1_3_.z(),
                  &position_controller_.controller_parameters_.hatKp1_3_.z());

            GetRosParameter(private_nh_, "hatKq0_3/x",
                  position_controller_.controller_parameters_.hatKq0_3_.x(),
                  &position_controller_.controller_parameters_.hatKq0_3_.x());
            GetRosParameter(private_nh_, "hatKq0_3/y",
                  position_controller_.controller_parameters_.hatKq0_3_.y(),
                  &position_controller_.controller_parameters_.hatKq0_3_.y());
            GetRosParameter(private_nh_, "hatKq0_3/z",
                  position_controller_.controller_parameters_.hatKq0_3_.z(),
                  &position_controller_.controller_parameters_.hatKq0_3_.z());

            GetRosParameter(private_nh_, "hatKq1_3/x",
                  position_controller_.controller_parameters_.hatKq1_3_.x(),
                  &position_controller_.controller_parameters_.hatKq1_3_.x());
            GetRosParameter(private_nh_, "hatKq1_3/y",
                  position_controller_.controller_parameters_.hatKq1_3_.y(),
                  &position_controller_.controller_parameters_.hatKq1_3_.y());
            GetRosParameter(private_nh_, "hatKq1_3/z",
                  position_controller_.controller_parameters_.hatKq1_3_.z(),
                  &position_controller_.controller_parameters_.hatKq1_3_.z());

            GetRosParameter(private_nh_, "hatKq2_3/x",
                  position_controller_.controller_parameters_.hatKq2_3_.x(),
                  &position_controller_.controller_parameters_.hatKq2_3_.x());
            GetRosParameter(private_nh_, "hatKq2_3/y",
                  position_controller_.controller_parameters_.hatKq2_3_.y(),
                  &position_controller_.controller_parameters_.hatKq2_3_.y());
            GetRosParameter(private_nh_, "hatKq2_3/z",
                  position_controller_.controller_parameters_.hatKq2_3_.z(),
                  &position_controller_.controller_parameters_.hatKq2_3_.z());

		GetRosParameter(private_nh_, "rho_p0_1/x",
                  position_controller_.controller_parameters_.rho_p0_1_.x(),
                  &position_controller_.controller_parameters_.rho_p0_1_.x());
		GetRosParameter(private_nh_, "rho_p0_1/y",
                  position_controller_.controller_parameters_.rho_p0_1_.y(),
                  &position_controller_.controller_parameters_.rho_p0_1_.y());
		GetRosParameter(private_nh_, "rho_p0_1/z",
                  position_controller_.controller_parameters_.rho_p0_1_.z(),
                  &position_controller_.controller_parameters_.rho_p0_1_.z());

		GetRosParameter(private_nh_, "rho_p1_1/x",
                  position_controller_.controller_parameters_.rho_p1_1_.x(),
                  &position_controller_.controller_parameters_.rho_p1_1_.x());
		GetRosParameter(private_nh_, "rho_p1_1/y",
                  position_controller_.controller_parameters_.rho_p1_1_.y(),
                  &position_controller_.controller_parameters_.rho_p1_1_.y());
		GetRosParameter(private_nh_, "rho_p1_1/z",
                  position_controller_.controller_parameters_.rho_p1_1_.z(),
                  &position_controller_.controller_parameters_.rho_p1_1_.z());

		GetRosParameter(private_nh_, "rho_q0_1/x",
                  position_controller_.controller_parameters_.rho_q0_1_.x(),
                  &position_controller_.controller_parameters_.rho_q0_1_.x());
		GetRosParameter(private_nh_, "rho_q0_1/y",
                  position_controller_.controller_parameters_.rho_q0_1_.y(),
                  &position_controller_.controller_parameters_.rho_q0_1_.y());
		GetRosParameter(private_nh_, "rho_q0_1/z",
                  position_controller_.controller_parameters_.rho_q0_1_.z(),
                  &position_controller_.controller_parameters_.rho_q0_1_.z());

		GetRosParameter(private_nh_, "rho_q1_1/x",
                  position_controller_.controller_parameters_.rho_q1_1_.x(),
                  &position_controller_.controller_parameters_.rho_q1_1_.x());
		GetRosParameter(private_nh_, "rho_q1_1/y",
                  position_controller_.controller_parameters_.rho_q1_1_.y(),
                  &position_controller_.controller_parameters_.rho_q1_1_.y());
		GetRosParameter(private_nh_, "rho_q1_1/z",
                  position_controller_.controller_parameters_.rho_q1_1_.z(),
                  &position_controller_.controller_parameters_.rho_q1_1_.z());

		GetRosParameter(private_nh_, "rho_q2_1/x",
                  position_controller_.controller_parameters_.rho_q2_1_.x(),
                  &position_controller_.controller_parameters_.rho_q2_1_.x());
		GetRosParameter(private_nh_, "rho_q2_1/y",
                  position_controller_.controller_parameters_.rho_q2_1_.y(),
                  &position_controller_.controller_parameters_.rho_q2_1_.y());
		GetRosParameter(private_nh_, "rho_q2_1/z",
                  position_controller_.controller_parameters_.rho_q2_1_.z(),
                  &position_controller_.controller_parameters_.rho_q2_1_.z());

		GetRosParameter(private_nh_, "rho_p0_2/x",
                  position_controller_.controller_parameters_.rho_p0_2_.x(),
                  &position_controller_.controller_parameters_.rho_p0_2_.x());
		GetRosParameter(private_nh_, "rho_p0_2/y",
                  position_controller_.controller_parameters_.rho_p0_2_.y(),
                  &position_controller_.controller_parameters_.rho_p0_2_.y());
		GetRosParameter(private_nh_, "rho_p0_2/z",
                  position_controller_.controller_parameters_.rho_p0_2_.z(),
                  &position_controller_.controller_parameters_.rho_p0_2_.z());

		GetRosParameter(private_nh_, "rho_p1_2/x",
                  position_controller_.controller_parameters_.rho_p1_2_.x(),
                  &position_controller_.controller_parameters_.rho_p1_2_.x());
		GetRosParameter(private_nh_, "rho_p1_2/y",
                  position_controller_.controller_parameters_.rho_p1_2_.y(),
                  &position_controller_.controller_parameters_.rho_p1_2_.y());
		GetRosParameter(private_nh_, "rho_p1_2/z",
                  position_controller_.controller_parameters_.rho_p1_2_.z(),
                  &position_controller_.controller_parameters_.rho_p1_2_.z());

		GetRosParameter(private_nh_, "rho_q0_2/x",
                  position_controller_.controller_parameters_.rho_q0_2_.x(),
                  &position_controller_.controller_parameters_.rho_q0_2_.x());
		GetRosParameter(private_nh_, "rho_q0_2/y",
                  position_controller_.controller_parameters_.rho_q0_2_.y(),
                  &position_controller_.controller_parameters_.rho_q0_2_.y());
		GetRosParameter(private_nh_, "rho_q0_2/z",
                  position_controller_.controller_parameters_.rho_q0_2_.z(),
                  &position_controller_.controller_parameters_.rho_q0_2_.z());

		GetRosParameter(private_nh_, "rho_q1_2/x",
                  position_controller_.controller_parameters_.rho_q1_2_.x(),
                  &position_controller_.controller_parameters_.rho_q1_2_.x());
		GetRosParameter(private_nh_, "rho_q1_2/y",
                  position_controller_.controller_parameters_.rho_q1_2_.y(),
                  &position_controller_.controller_parameters_.rho_q1_2_.y());
		GetRosParameter(private_nh_, "rho_q1_2/z",
                  position_controller_.controller_parameters_.rho_q1_2_.z(),
                  &position_controller_.controller_parameters_.rho_q1_2_.z());

		GetRosParameter(private_nh_, "rho_q2_2/x",
                  position_controller_.controller_parameters_.rho_q2_2_.x(),
                  &position_controller_.controller_parameters_.rho_q2_2_.x());
		GetRosParameter(private_nh_, "rho_q2_2/y",
                  position_controller_.controller_parameters_.rho_q2_2_.y(),
                  &position_controller_.controller_parameters_.rho_q2_2_.y());
		GetRosParameter(private_nh_, "rho_q2_2/z",
                  position_controller_.controller_parameters_.rho_q2_2_.z(),
                  &position_controller_.controller_parameters_.rho_q2_2_.z());

            GetRosParameter(private_nh_, "rho_p0_3/x",
                  position_controller_.controller_parameters_.rho_p0_3_.x(),
                  &position_controller_.controller_parameters_.rho_p0_3_.x());
            GetRosParameter(private_nh_, "rho_p0_3/y",
                  position_controller_.controller_parameters_.rho_p0_3_.y(),
                  &position_controller_.controller_parameters_.rho_p0_3_.y());
            GetRosParameter(private_nh_, "rho_p0_3/z",
                  position_controller_.controller_parameters_.rho_p0_3_.z(),
                  &position_controller_.controller_parameters_.rho_p0_3_.z());

            GetRosParameter(private_nh_, "rho_p1_3/x",
                  position_controller_.controller_parameters_.rho_p1_3_.x(),
                  &position_controller_.controller_parameters_.rho_p1_3_.x());
            GetRosParameter(private_nh_, "rho_p1_3/y",
                  position_controller_.controller_parameters_.rho_p1_3_.y(),
                  &position_controller_.controller_parameters_.rho_p1_3_.y());
            GetRosParameter(private_nh_, "rho_p1_3/z",
                  position_controller_.controller_parameters_.rho_p1_3_.z(),
                  &position_controller_.controller_parameters_.rho_p1_3_.z());

            GetRosParameter(private_nh_, "rho_q0_3/x",
                  position_controller_.controller_parameters_.rho_q0_3_.x(),
                  &position_controller_.controller_parameters_.rho_q0_3_.x());
            GetRosParameter(private_nh_, "rho_q0_3/y",
                  position_controller_.controller_parameters_.rho_q0_3_.y(),
                  &position_controller_.controller_parameters_.rho_q0_3_.y());
            GetRosParameter(private_nh_, "rho_q0_3/z",
                  position_controller_.controller_parameters_.rho_q0_3_.z(),
                  &position_controller_.controller_parameters_.rho_q0_3_.z());

            GetRosParameter(private_nh_, "rho_q1_3/x",
                  position_controller_.controller_parameters_.rho_q1_3_.x(),
                  &position_controller_.controller_parameters_.rho_q1_3_.x());
            GetRosParameter(private_nh_, "rho_q1_3/y",
                  position_controller_.controller_parameters_.rho_q1_3_.y(),
                  &position_controller_.controller_parameters_.rho_q1_3_.y());
            GetRosParameter(private_nh_, "rho_q1_3/z",
                  position_controller_.controller_parameters_.rho_q1_3_.z(),
                  &position_controller_.controller_parameters_.rho_q1_3_.z());

            GetRosParameter(private_nh_, "rho_q2_3/x",
                  position_controller_.controller_parameters_.rho_q2_3_.x(),
                  &position_controller_.controller_parameters_.rho_q2_3_.x());
            GetRosParameter(private_nh_, "rho_q2_3/y",
                  position_controller_.controller_parameters_.rho_q2_3_.y(),
                  &position_controller_.controller_parameters_.rho_q2_3_.y());
            GetRosParameter(private_nh_, "rho_q2_3/z",
                  position_controller_.controller_parameters_.rho_q2_3_.z(),
                  &position_controller_.controller_parameters_.rho_q2_3_.z());

		GetRosParameter(private_nh_, "beta_p0_1/x",
                  position_controller_.controller_parameters_.beta_p0_1_.x(),
                  &position_controller_.controller_parameters_.beta_p0_1_.x());
		GetRosParameter(private_nh_, "beta_p0_1/y",
                  position_controller_.controller_parameters_.beta_p0_1_.y(),
                  &position_controller_.controller_parameters_.beta_p0_1_.y());
		GetRosParameter(private_nh_, "beta_p0_1/z",
                  position_controller_.controller_parameters_.beta_p0_1_.z(),
                  &position_controller_.controller_parameters_.beta_p0_1_.z());

		GetRosParameter(private_nh_, "beta_p1_1/x",
                  position_controller_.controller_parameters_.beta_p1_1_.x(),
                  &position_controller_.controller_parameters_.beta_p1_1_.x());
		GetRosParameter(private_nh_, "beta_p1_1/y",
                  position_controller_.controller_parameters_.beta_p1_1_.y(),
                  &position_controller_.controller_parameters_.beta_p1_1_.y());
		GetRosParameter(private_nh_, "beta_p1_1/z",
                  position_controller_.controller_parameters_.beta_p1_1_.z(),
                  &position_controller_.controller_parameters_.beta_p1_1_.z());

		GetRosParameter(private_nh_, "beta_q0_1/x",
                  position_controller_.controller_parameters_.beta_q0_1_.x(),
                  &position_controller_.controller_parameters_.beta_q0_1_.x());
		GetRosParameter(private_nh_, "beta_q0_1/y",
                  position_controller_.controller_parameters_.beta_q0_1_.y(),
                  &position_controller_.controller_parameters_.beta_q0_1_.y());
		GetRosParameter(private_nh_, "beta_q0_1/z",
                  position_controller_.controller_parameters_.beta_q0_1_.z(),
                  &position_controller_.controller_parameters_.beta_q0_1_.z());

		GetRosParameter(private_nh_, "beta_q1_1/x",
                  position_controller_.controller_parameters_.beta_q1_1_.x(),
                  &position_controller_.controller_parameters_.beta_q1_1_.x());
		GetRosParameter(private_nh_, "beta_q1_1/y",
                  position_controller_.controller_parameters_.beta_q1_1_.y(),
                  &position_controller_.controller_parameters_.beta_q1_1_.y());
		GetRosParameter(private_nh_, "beta_q1_1/z",
                  position_controller_.controller_parameters_.beta_q1_1_.z(),
                  &position_controller_.controller_parameters_.beta_q1_1_.z());

		GetRosParameter(private_nh_, "beta_q2_1/x",
                  position_controller_.controller_parameters_.beta_q2_1_.x(),
                  &position_controller_.controller_parameters_.beta_q2_1_.x());
		GetRosParameter(private_nh_, "beta_q2_1/y",
                  position_controller_.controller_parameters_.beta_q2_1_.y(),
                  &position_controller_.controller_parameters_.beta_q2_1_.y());
		GetRosParameter(private_nh_, "beta_q2_1/z",
                  position_controller_.controller_parameters_.beta_q2_1_.z(),
                  &position_controller_.controller_parameters_.beta_q2_1_.z());

		GetRosParameter(private_nh_, "beta_p0_2/x",
                  position_controller_.controller_parameters_.beta_p0_2_.x(),
                  &position_controller_.controller_parameters_.beta_p0_2_.x());
		GetRosParameter(private_nh_, "beta_p0_2/y",
                  position_controller_.controller_parameters_.beta_p0_2_.y(),
                  &position_controller_.controller_parameters_.beta_p0_2_.y());
		GetRosParameter(private_nh_, "beta_p0_2/z",
                  position_controller_.controller_parameters_.beta_p0_2_.z(),
                  &position_controller_.controller_parameters_.beta_p0_2_.z());

		GetRosParameter(private_nh_, "beta_p1_2/x",
                  position_controller_.controller_parameters_.beta_p1_2_.x(),
                  &position_controller_.controller_parameters_.beta_p1_2_.x());
		GetRosParameter(private_nh_, "beta_p1_2/y",
                  position_controller_.controller_parameters_.beta_p1_2_.y(),
                  &position_controller_.controller_parameters_.beta_p1_2_.y());
		GetRosParameter(private_nh_, "beta_p1_2/z",
                  position_controller_.controller_parameters_.beta_p1_2_.z(),
                  &position_controller_.controller_parameters_.beta_p1_2_.z());

		GetRosParameter(private_nh_, "beta_q0_2/x",
                  position_controller_.controller_parameters_.beta_q0_2_.x(),
                  &position_controller_.controller_parameters_.beta_q0_2_.x());
		GetRosParameter(private_nh_, "beta_q0_2/y",
                  position_controller_.controller_parameters_.beta_q0_2_.y(),
                  &position_controller_.controller_parameters_.beta_q0_2_.y());
		GetRosParameter(private_nh_, "beta_q0_2/z",
                  position_controller_.controller_parameters_.beta_q0_2_.z(),
                  &position_controller_.controller_parameters_.beta_q0_2_.z());

		GetRosParameter(private_nh_, "beta_q1_2/x",
                  position_controller_.controller_parameters_.beta_q1_2_.x(),
                  &position_controller_.controller_parameters_.beta_q1_2_.x());
		GetRosParameter(private_nh_, "beta_q1_2/y",
                  position_controller_.controller_parameters_.beta_q1_2_.y(),
                  &position_controller_.controller_parameters_.beta_q1_2_.y());
		GetRosParameter(private_nh_, "beta_q1_2/z",
                  position_controller_.controller_parameters_.beta_q1_2_.z(),
                  &position_controller_.controller_parameters_.beta_q1_2_.z());

		GetRosParameter(private_nh_, "beta_q2_2/x",
                  position_controller_.controller_parameters_.beta_q2_2_.x(),
                  &position_controller_.controller_parameters_.beta_q2_2_.x());
		GetRosParameter(private_nh_, "beta_q2_2/y",
                  position_controller_.controller_parameters_.beta_q2_2_.y(),
                  &position_controller_.controller_parameters_.beta_q2_2_.y());
		GetRosParameter(private_nh_, "beta_q2_2/z",
                  position_controller_.controller_parameters_.beta_q2_2_.z(),
                  &position_controller_.controller_parameters_.beta_q2_2_.z());

            GetRosParameter(private_nh_, "beta_p0_3/x",
                  position_controller_.controller_parameters_.beta_p0_3_.x(),
                  &position_controller_.controller_parameters_.beta_p0_3_.x());
            GetRosParameter(private_nh_, "beta_p0_3/y",
                  position_controller_.controller_parameters_.beta_p0_3_.y(),
                  &position_controller_.controller_parameters_.beta_p0_3_.y());
            GetRosParameter(private_nh_, "beta_p0_3/z",
                  position_controller_.controller_parameters_.beta_p0_3_.z(),
                  &position_controller_.controller_parameters_.beta_p0_3_.z());

            GetRosParameter(private_nh_, "beta_p1_3/x",
                  position_controller_.controller_parameters_.beta_p1_3_.x(),
                  &position_controller_.controller_parameters_.beta_p1_3_.x());
            GetRosParameter(private_nh_, "beta_p1_3/y",
                  position_controller_.controller_parameters_.beta_p1_3_.y(),
                  &position_controller_.controller_parameters_.beta_p1_3_.y());
            GetRosParameter(private_nh_, "beta_p1_3/z",
                  position_controller_.controller_parameters_.beta_p1_3_.z(),
                  &position_controller_.controller_parameters_.beta_p1_3_.z());

            GetRosParameter(private_nh_, "beta_q0_3/x",
                  position_controller_.controller_parameters_.beta_q0_3_.x(),
                  &position_controller_.controller_parameters_.beta_q0_3_.x());
            GetRosParameter(private_nh_, "beta_q0_3/y",
                  position_controller_.controller_parameters_.beta_q0_3_.y(),
                  &position_controller_.controller_parameters_.beta_q0_3_.y());
            GetRosParameter(private_nh_, "beta_q0_3/z",
                  position_controller_.controller_parameters_.beta_q0_3_.z(),
                  &position_controller_.controller_parameters_.beta_q0_3_.z());

            GetRosParameter(private_nh_, "beta_q1_3/x",
                  position_controller_.controller_parameters_.beta_q1_3_.x(),
                  &position_controller_.controller_parameters_.beta_q1_3_.x());
            GetRosParameter(private_nh_, "beta_q1_3/y",
                  position_controller_.controller_parameters_.beta_q1_3_.y(),
                  &position_controller_.controller_parameters_.beta_q1_3_.y());
            GetRosParameter(private_nh_, "beta_q1_3/z",
                  position_controller_.controller_parameters_.beta_q1_3_.z(),
                  &position_controller_.controller_parameters_.beta_q1_3_.z());

            GetRosParameter(private_nh_, "beta_q2_3/x",
                  position_controller_.controller_parameters_.beta_q2_3_.x(),
                  &position_controller_.controller_parameters_.beta_q2_3_.x());
            GetRosParameter(private_nh_, "beta_q2_3/y",
                  position_controller_.controller_parameters_.beta_q2_3_.y(),
                  &position_controller_.controller_parameters_.beta_q2_3_.y());
            GetRosParameter(private_nh_, "beta_q2_3/z",
                  position_controller_.controller_parameters_.beta_q2_3_.z(),
                  &position_controller_.controller_parameters_.beta_q2_3_.z());

		GetRosParameter(private_nh_, "nu_p0_1/x",
                  position_controller_.controller_parameters_.nu_p0_1_.x(),
                  &position_controller_.controller_parameters_.nu_p0_1_.x());
		GetRosParameter(private_nh_, "nu_p0_1/y",
                  position_controller_.controller_parameters_.nu_p0_1_.y(),
                  &position_controller_.controller_parameters_.nu_p0_1_.y());
		GetRosParameter(private_nh_, "nu_p0_1/z",
                  position_controller_.controller_parameters_.nu_p0_1_.z(),
                  &position_controller_.controller_parameters_.nu_p0_1_.z());

		GetRosParameter(private_nh_, "nu_p1_1/x",
                  position_controller_.controller_parameters_.nu_p1_1_.x(),
                  &position_controller_.controller_parameters_.nu_p1_1_.x());
		GetRosParameter(private_nh_, "nu_p1_1/y",
                  position_controller_.controller_parameters_.nu_p1_1_.y(),
                  &position_controller_.controller_parameters_.nu_p1_1_.y());
		GetRosParameter(private_nh_, "nu_p1_1/z",
                  position_controller_.controller_parameters_.nu_p1_1_.z(),
                  &position_controller_.controller_parameters_.nu_p1_1_.z());

		GetRosParameter(private_nh_, "nu_p1_2/x",
                  position_controller_.controller_parameters_.nu_p1_2_.x(),
                  &position_controller_.controller_parameters_.nu_p1_2_.x());
		GetRosParameter(private_nh_, "nu_p1_2/y",
                  position_controller_.controller_parameters_.nu_p1_2_.y(),
                  &position_controller_.controller_parameters_.nu_p1_2_.y());
		GetRosParameter(private_nh_, "nu_p1_2/z",
                  position_controller_.controller_parameters_.nu_p1_2_.z(),
                  &position_controller_.controller_parameters_.nu_p1_2_.z());

		GetRosParameter(private_nh_, "nu_q0_1/x",
                  position_controller_.controller_parameters_.nu_q0_1_.x(),
                  &position_controller_.controller_parameters_.nu_q0_1_.x());
		GetRosParameter(private_nh_, "nu_q0_1/y",
                  position_controller_.controller_parameters_.nu_q0_1_.y(),
                  &position_controller_.controller_parameters_.nu_q0_1_.y());
		GetRosParameter(private_nh_, "nu_q0_1/z",
                  position_controller_.controller_parameters_.nu_q0_1_.z(),
                  &position_controller_.controller_parameters_.nu_q0_1_.z());

		GetRosParameter(private_nh_, "nu_q1_1/x",
                  position_controller_.controller_parameters_.nu_q1_1_.x(),
                  &position_controller_.controller_parameters_.nu_q1_1_.x());
		GetRosParameter(private_nh_, "nu_q1_1/y",
                  position_controller_.controller_parameters_.nu_q1_1_.y(),
                  &position_controller_.controller_parameters_.nu_q1_1_.y());
		GetRosParameter(private_nh_, "nu_q1_1/z",
                  position_controller_.controller_parameters_.nu_q1_1_.z(),
                  &position_controller_.controller_parameters_.nu_q1_1_.z());

		GetRosParameter(private_nh_, "nu_q2_1/x",
                  position_controller_.controller_parameters_.nu_q2_1_.x(),
                  &position_controller_.controller_parameters_.nu_q2_1_.x());
		GetRosParameter(private_nh_, "nu_q2_1/y",
                  position_controller_.controller_parameters_.nu_q2_1_.y(),
                  &position_controller_.controller_parameters_.nu_q2_1_.y());
		GetRosParameter(private_nh_, "nu_q2_1/z",
                  position_controller_.controller_parameters_.nu_q2_1_.z(),
                  &position_controller_.controller_parameters_.nu_q2_1_.z());

		GetRosParameter(private_nh_, "nu_p0_2/x",
                  position_controller_.controller_parameters_.nu_p0_2_.x(),
                  &position_controller_.controller_parameters_.nu_p0_2_.x());
		GetRosParameter(private_nh_, "nu_p0_2/y",
                  position_controller_.controller_parameters_.nu_p0_2_.y(),
                  &position_controller_.controller_parameters_.nu_p0_2_.y());
		GetRosParameter(private_nh_, "nu_p0_2/z",
                  position_controller_.controller_parameters_.nu_p0_2_.z(),
                  &position_controller_.controller_parameters_.nu_p0_2_.z());

		GetRosParameter(private_nh_, "nu_q0_2/x",
                  position_controller_.controller_parameters_.nu_q0_2_.x(),
                  &position_controller_.controller_parameters_.nu_q0_2_.x());
		GetRosParameter(private_nh_, "nu_q0_2/y",
                  position_controller_.controller_parameters_.nu_q0_2_.y(),
                  &position_controller_.controller_parameters_.nu_q0_2_.y());
		GetRosParameter(private_nh_, "nu_q0_2/z",
                  position_controller_.controller_parameters_.nu_q0_2_.z(),
                  &position_controller_.controller_parameters_.nu_q0_2_.z());

		GetRosParameter(private_nh_, "nu_q1_2/x",
                  position_controller_.controller_parameters_.nu_q1_2_.x(),
                  &position_controller_.controller_parameters_.nu_q1_2_.x());
		GetRosParameter(private_nh_, "nu_q1_2/y",
                  position_controller_.controller_parameters_.nu_q1_2_.y(),
                  &position_controller_.controller_parameters_.nu_q1_2_.y());
		GetRosParameter(private_nh_, "nu_q1_2/z",
                  position_controller_.controller_parameters_.nu_q1_2_.z(),
                  &position_controller_.controller_parameters_.nu_q1_2_.z());

		GetRosParameter(private_nh_, "nu_q2_2/x",
                  position_controller_.controller_parameters_.nu_q2_2_.x(),
                  &position_controller_.controller_parameters_.nu_q2_2_.x());
		GetRosParameter(private_nh_, "nu_q2_2/y",
                  position_controller_.controller_parameters_.nu_q2_2_.y(),
                  &position_controller_.controller_parameters_.nu_q2_2_.y());
		GetRosParameter(private_nh_, "nu_q2_2/z",
                  position_controller_.controller_parameters_.nu_q2_2_.z(),
                  &position_controller_.controller_parameters_.beta_q2_2_.z());

            GetRosParameter(private_nh_, "nu_p0_3/x",
                  position_controller_.controller_parameters_.nu_p0_3_.x(),
                  &position_controller_.controller_parameters_.nu_p0_3_.x());
            GetRosParameter(private_nh_, "nu_p0_3/y",
                  position_controller_.controller_parameters_.nu_p0_3_.y(),
                  &position_controller_.controller_parameters_.nu_p0_3_.y());
            GetRosParameter(private_nh_, "nu_p0_3/z",
                  position_controller_.controller_parameters_.nu_p0_3_.z(),
                  &position_controller_.controller_parameters_.nu_p0_3_.z());

            GetRosParameter(private_nh_, "nu_q0_3/x",
                  position_controller_.controller_parameters_.nu_q0_3_.x(),
                  &position_controller_.controller_parameters_.nu_q0_3_.x());
            GetRosParameter(private_nh_, "nu_q0_3/y",
                  position_controller_.controller_parameters_.nu_q0_3_.y(),
                  &position_controller_.controller_parameters_.nu_q0_3_.y());
            GetRosParameter(private_nh_, "nu_q0_3/z",
                  position_controller_.controller_parameters_.nu_q0_3_.z(),
                  &position_controller_.controller_parameters_.nu_q0_3_.z());

            GetRosParameter(private_nh_, "nu_q1_3/x",
                  position_controller_.controller_parameters_.nu_q1_3_.x(),
                  &position_controller_.controller_parameters_.nu_q1_3_.x());
            GetRosParameter(private_nh_, "nu_q1_3/y",
                  position_controller_.controller_parameters_.nu_q1_3_.y(),
                  &position_controller_.controller_parameters_.nu_q1_3_.y());
            GetRosParameter(private_nh_, "nu_q1_3/z",
                  position_controller_.controller_parameters_.nu_q1_3_.z(),
                  &position_controller_.controller_parameters_.nu_q1_3_.z());

            GetRosParameter(private_nh_, "nu_q2_3/x",
                  position_controller_.controller_parameters_.nu_q2_3_.x(),
                  &position_controller_.controller_parameters_.nu_q2_3_.x());
            GetRosParameter(private_nh_, "nu_q2_3/y",
                  position_controller_.controller_parameters_.nu_q2_3_.y(),
                  &position_controller_.controller_parameters_.nu_q2_3_.y());
            GetRosParameter(private_nh_, "nu_q2_3/z",
                  position_controller_.controller_parameters_.nu_q2_3_.z(),
                  &position_controller_.controller_parameters_.beta_q2_3_.z());

		GetRosParameter(private_nh_, "lam_p_1/x",
                  position_controller_.controller_parameters_.lam_p_1_.x(),
                  &position_controller_.controller_parameters_.lam_p_1_.x());
		GetRosParameter(private_nh_, "lam_p_1/y",
                  position_controller_.controller_parameters_.lam_p_1_.y(),
                  &position_controller_.controller_parameters_.lam_p_1_.y());
		GetRosParameter(private_nh_, "lam_p_1/z",
                  position_controller_.controller_parameters_.lam_p_1_.z(),
                  &position_controller_.controller_parameters_.lam_p_1_.z());

		GetRosParameter(private_nh_, "lam_q_1/x",
                  position_controller_.controller_parameters_.lam_q_1_.x(),
                  &position_controller_.controller_parameters_.lam_q_1_.x());
		GetRosParameter(private_nh_, "lam_q_1/y",
                  position_controller_.controller_parameters_.lam_q_1_.y(),
                  &position_controller_.controller_parameters_.lam_q_1_.y());
		GetRosParameter(private_nh_, "lam_q_1/z",
                  position_controller_.controller_parameters_.lam_q_1_.z(),
                  &position_controller_.controller_parameters_.lam_q_1_.z());

		GetRosParameter(private_nh_, "lam_p_2/x",
                  position_controller_.controller_parameters_.lam_p_2_.x(),
                  &position_controller_.controller_parameters_.lam_p_2_.x());
		GetRosParameter(private_nh_, "lam_p_2/y",
                  position_controller_.controller_parameters_.lam_p_2_.y(),
                  &position_controller_.controller_parameters_.lam_p_2_.y());
		GetRosParameter(private_nh_, "lam_p_2/z",
                  position_controller_.controller_parameters_.lam_p_2_.z(),
                  &position_controller_.controller_parameters_.lam_p_2_.z());

		GetRosParameter(private_nh_, "lam_q_2/x",
                  position_controller_.controller_parameters_.lam_q_2_.x(),
                  &position_controller_.controller_parameters_.lam_q_2_.x());
		GetRosParameter(private_nh_, "lam_q_2/y",
                  position_controller_.controller_parameters_.lam_q_2_.y(),
                  &position_controller_.controller_parameters_.lam_q_2_.y());
		GetRosParameter(private_nh_, "lam_q_2/z",
                  position_controller_.controller_parameters_.lam_q_2_.z(),
                  &position_controller_.controller_parameters_.lam_q_2_.z());

            GetRosParameter(private_nh_, "lam_p_3/x",
                  position_controller_.controller_parameters_.lam_p_3_.x(),
                  &position_controller_.controller_parameters_.lam_p_3_.x());
            GetRosParameter(private_nh_, "lam_p_3/y",
                  position_controller_.controller_parameters_.lam_p_3_.y(),
                  &position_controller_.controller_parameters_.lam_p_3_.y());
            GetRosParameter(private_nh_, "lam_p_3/z",
                  position_controller_.controller_parameters_.lam_p_3_.z(),
                  &position_controller_.controller_parameters_.lam_p_3_.z());

            GetRosParameter(private_nh_, "lam_q_3/x",
                  position_controller_.controller_parameters_.lam_q_3_.x(),
                  &position_controller_.controller_parameters_.lam_q_3_.x());
            GetRosParameter(private_nh_, "lam_q_3/y",
                  position_controller_.controller_parameters_.lam_q_3_.y(),
                  &position_controller_.controller_parameters_.lam_q_3_.y());
            GetRosParameter(private_nh_, "lam_q_3/z",
                  position_controller_.controller_parameters_.lam_q_3_.z(),
                  &position_controller_.controller_parameters_.lam_q_3_.z());

		GetRosParameter(private_nh_, "alphaM",
                  position_controller_.controller_parameters_.alpha_m_,
                  &position_controller_.controller_parameters_.alpha_m_);

		GetRosParameter(private_nh_, "hatM_1",
                  position_controller_.controller_parameters_.hatM_1_,
                  &position_controller_.controller_parameters_.hatM_1_);

		GetRosParameter(private_nh_, "hatM_2",
                  position_controller_.controller_parameters_.hatM_2_,
                  &position_controller_.controller_parameters_.hatM_2_);

            GetRosParameter(private_nh_, "hatM_3",
                  position_controller_.controller_parameters_.hatM_3_,
                  &position_controller_.controller_parameters_.hatM_3_);

		GetRosParameter(private_nh_, "var_pi_p",
                  position_controller_.controller_parameters_.var_pi_p_,
                  &position_controller_.controller_parameters_.var_pi_p_);

		GetRosParameter(private_nh_, "var_pi_q",
                  position_controller_.controller_parameters_.var_pi_q_,
                  &position_controller_.controller_parameters_.var_pi_q_);
		
		// ROS_INFO_STREAM("Rotor size:" << position_controller_.vehicle_parameters_.rotor_configuration_.rotors.size());
		GetVehicleParameters(private_nh_, &position_controller_.vehicle_parameters_);
		// ROS_INFO_STREAM("Rotor size:" << position_controller_.vehicle_parameters_.rotor_configuration_.rotors.size());

		position_controller_.InitializeParameters();
	}

	void SacPositionControllerNode::MdjTrajCallback(
		const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {

		// Clear all pending commands.
		command_timer_.stop();
		commands_.clear();
		command_waiting_times_.clear();

		const size_t n_commands = msg->points.size();

		if(n_commands < 1){
		  ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
		  return;
		}	


		mav_msgs::EigenTrajectoryPoint eigen_reference;
		mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
		commands_.push_front(eigen_reference);	

		for (size_t i = 1; i < n_commands; ++i) {
		  const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
		  const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];	
		  mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);	
		  commands_.push_back(eigen_reference);
		  command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
		}	

		// We can trigger the first command immediately.
		position_controller_.SetTrajectoryPoint(commands_.front());
		commands_.pop_front();	
		if (n_commands > 1) {
		  command_timer_.setPeriod(command_waiting_times_.front());
		  command_waiting_times_.pop_front();
		  command_timer_.start();
		}
	}

	void SacPositionControllerNode::OdometryCallback(
		const nav_msgs::OdometryConstPtr& odometry_msg) {
		ROS_INFO_ONCE("SacPositionController got first odometry message.");

		EigenOdometry odometry;
		eigenOdometryFromMsg(odometry_msg, &odometry);
		position_controller_.SetOdometry(odometry);

		Eigen::VectorXd ref_rotor_velocities;
		position_controller_.CalculateRotorVelocities(&ref_rotor_velocities, &data_out_);

		// Todo(ffurrer): Do this in the conversions header.
		mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

		actuator_msg->angular_velocities.clear();
		for (int i = 0; i < ref_rotor_velocities.size(); i++)
		  actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
		actuator_msg->header.stamp = odometry_msg->header.stamp;
		data_out_.header.stamp = odometry_msg->header.stamp;

		motor_vel_pub_.publish(actuator_msg);
		plot_data_pub_.publish(data_out_);

	}

	void SacPositionControllerNode::TimedCallback(const ros::TimerEvent& e) {
		if(commands_.empty()){
		  ROS_WARN("Commands empty, this should not happen here");
		  return;
		}
		
		const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
		position_controller_.SetTrajectoryPoint(commands_.front());
		commands_.pop_front();
		command_timer_.stop();

		if(!command_waiting_times_.empty()){
		  command_timer_.setPeriod(command_waiting_times_.front());
		  command_waiting_times_.pop_front();
		  command_timer_.start();
		}
	}


	void SacPositionControllerNode::PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
	  ROS_INFO_ONCE("PidPositionController got first pose message.");

	  eigenOdometryFromPoseCovMsg(msg, &odometry);  

	  position_controller_.SetOdometry(odometry);

	  Eigen::VectorXd ref_rotor_velocities;
	  position_controller_.CalculateRotorVelocities(&ref_rotor_velocities, &data_out_);

	  // Todo(ffurrer): Do this in the conversions header.
	  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

	  actuator_msg->angular_velocities.clear();
	  for (int i = 0; i < ref_rotor_velocities.size(); i++)
	    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
	  actuator_msg->header.stamp = msg->header.stamp;
	  data_out_.header.stamp = msg->header.stamp;

	  motor_vel_pub_.publish(actuator_msg);
	  plot_data_pub_.publish(data_out_);
	}

	void SacPositionControllerNode::StateCallback(const std_msgs::UInt8::ConstPtr& state_msg){
		position_controller_.SetState(state_msg->data);
	}



}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sac_position_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rrc_control::SacPositionControllerNode sac_position_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}