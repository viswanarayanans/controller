#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "autde_position_controller_node.h"

#include "rrc_control/parameters_ros.h"

namespace rrc_control {
	AuTdePositionControllerNode::AuTdePositionControllerNode(
		const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
	:nh_(nh), private_nh_(private_nh) {
		InitializeParams();

		cmd_mdj_traj_sub_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
			&AuTdePositionControllerNode::MdjTrajCallback, this);
		cmd_odom_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
			&AuTdePositionControllerNode::OdometryCallback, this);


		pose_sub_ = nh_.subscribe("/mavros/vision_pose/pose", 1, 
                            &AuTdePositionControllerNode::PoseCallback, this);


		motor_vel_pub_ = nh_.advertise<mav_msgs::Actuators>(
			mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  		plot_data_pub_ = nh_.advertise<msg_check::PlotDataMsg>("/data_out", 1);


		command_timer_ = nh_.createTimer(ros::Duration(0), &AuTdePositionControllerNode::TimedCallback, this,
			true, false);
	}

	AuTdePositionControllerNode::~AuTdePositionControllerNode() {}

	void AuTdePositionControllerNode::InitializeParams() {
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

		GetRosParameter(private_nh_, "Kpp/x",
                  position_controller_.controller_parameters_.Kpp_.x(),
                  &position_controller_.controller_parameters_.Kpp_.x());
		GetRosParameter(private_nh_, "Kpp/y",
                  position_controller_.controller_parameters_.Kpp_.y(),
                  &position_controller_.controller_parameters_.Kpp_.y());
		GetRosParameter(private_nh_, "Kpp/z",
                  position_controller_.controller_parameters_.Kpp_.z(),
                  &position_controller_.controller_parameters_.Kpp_.z());

		GetRosParameter(private_nh_, "Kpq/x",
                  position_controller_.controller_parameters_.Kpq_.x(),
                  &position_controller_.controller_parameters_.Kpq_.x());
		GetRosParameter(private_nh_, "Kpq/y",
                  position_controller_.controller_parameters_.Kpq_.y(),
                  &position_controller_.controller_parameters_.Kpq_.y());
		GetRosParameter(private_nh_, "Kpq/z",
                  position_controller_.controller_parameters_.Kpq_.z(),
                  &position_controller_.controller_parameters_.Kpq_.z());

		GetRosParameter(private_nh_, "Kdp/x",
                  position_controller_.controller_parameters_.Kdp_.x(),
                  &position_controller_.controller_parameters_.Kdp_.x());
		GetRosParameter(private_nh_, "Kdp/y",
                  position_controller_.controller_parameters_.Kdp_.y(),
                  &position_controller_.controller_parameters_.Kdp_.y());
		GetRosParameter(private_nh_, "Kdp/z",
                  position_controller_.controller_parameters_.Kdp_.z(),
                  &position_controller_.controller_parameters_.Kdp_.z());

		GetRosParameter(private_nh_, "Kdq/x",
                  position_controller_.controller_parameters_.Kdq_.x(),
                  &position_controller_.controller_parameters_.Kdq_.x());
		GetRosParameter(private_nh_, "Kdq/y",
                  position_controller_.controller_parameters_.Kdq_.y(),
                  &position_controller_.controller_parameters_.Kdq_.y());
		GetRosParameter(private_nh_, "Kdq/z",
                  position_controller_.controller_parameters_.Kdq_.z(),
                  &position_controller_.controller_parameters_.Kdq_.z());

		GetRosParameter(private_nh_, "alphaM",
                  position_controller_.controller_parameters_.alpha_m_,
                  &position_controller_.controller_parameters_.alpha_m_);

		GetRosParameter(private_nh_, "hatM",
                  position_controller_.controller_parameters_.hatM_,
                  &position_controller_.controller_parameters_.hatM_);

		GetRosParameter(private_nh_, "alpha_p0",
                  position_controller_.controller_parameters_.alpha_p0_,
                  &position_controller_.controller_parameters_.alpha_p0_);

		GetRosParameter(private_nh_, "alpha_p1",
                  position_controller_.controller_parameters_.alpha_p1_,
                  &position_controller_.controller_parameters_.alpha_p1_);

		GetRosParameter(private_nh_, "alpha_q0",
                  position_controller_.controller_parameters_.alpha_q0_,
                  &position_controller_.controller_parameters_.alpha_q0_);

		GetRosParameter(private_nh_, "alpha_q1",
                  position_controller_.controller_parameters_.alpha_p1_,
                  &position_controller_.controller_parameters_.alpha_p1_);

		GetRosParameter(private_nh_, "hatKp_0",
                  position_controller_.controller_parameters_.hatKp_0_,
                  &position_controller_.controller_parameters_.hatKp_0_);

		GetRosParameter(private_nh_, "hatKp_1",
                  position_controller_.controller_parameters_.hatKp_1_,
                  &position_controller_.controller_parameters_.hatKp_1_);

		GetRosParameter(private_nh_, "hatKq_0",
                  position_controller_.controller_parameters_.hatKq_0_,
                  &position_controller_.controller_parameters_.hatKq_0_);

		GetRosParameter(private_nh_, "hatKq_1",
                  position_controller_.controller_parameters_.hatKq_1_,
                  &position_controller_.controller_parameters_.hatKq_1_);

		GetRosParameter(private_nh_, "gamma_p",
                  position_controller_.controller_parameters_.gamma_p_,
                  &position_controller_.controller_parameters_.gamma_p_);

		GetRosParameter(private_nh_, "gamma_q",
                  position_controller_.controller_parameters_.gamma_q_,
                  &position_controller_.controller_parameters_.gamma_q_);

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

	void AuTdePositionControllerNode::MdjTrajCallback(
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

	void AuTdePositionControllerNode::OdometryCallback(
		const nav_msgs::OdometryConstPtr& odometry_msg) {
		ROS_INFO_ONCE("AuTdePositionController got first odometry message.");

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
		comm_.sendSerial(ref_rotor_velocities);
		plot_data_pub_.publish(data_out_);

	}

	void AuTdePositionControllerNode::TimedCallback(const ros::TimerEvent& e) {
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


	void AuTdePositionControllerNode::PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
	  ROS_INFO_ONCE("PidPositionController got first pose message.");

	  eigenOdometryFromPoseMsg(msg, &odometry);  

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
	  // comm_.sendSerial(ref_rotor_velocities);
	  plot_data_pub_.publish(data_out_);
	}



}

int main(int argc, char** argv) {
  ros::init(argc, argv, "autde_position_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rrc_control::AuTdePositionControllerNode autde_position_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}