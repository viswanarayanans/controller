#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "smc_position_controller_node.h"

#include "rrc_control/parameters_ros.h"

namespace rrc_control {
	SmcPositionControllerNode::SmcPositionControllerNode(
		const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
	:nh_(nh), private_nh_(private_nh) {
		InitializeParams();

		cmd_mdj_traj_sub_ = nh_.subscribe(mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
			&SmcPositionControllerNode::MdjTrajCallback, this);
		cmd_odom_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
			&SmcPositionControllerNode::OdometryCallback, this);

		pose_sub_ = nh_.subscribe("/mavros/vision_pose/pose", 1, 
                            &SmcPositionControllerNode::PoseCallback, this);

		motor_vel_pub_ = nh_.advertise<mav_msgs::Actuators>(
			mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  		plot_data_pub_ = nh_.advertise<msg_check::PlotDataMsg>("/data_out", 1);


		command_timer_ = nh_.createTimer(ros::Duration(0), &SmcPositionControllerNode::TimedCallback, this,
			true, false);
	}

	SmcPositionControllerNode::~SmcPositionControllerNode() {}

	void SmcPositionControllerNode::InitializeParams() {
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

		// GetRosParameter(private_nh_, "inertia/xx",
  //                 position_controller_.controller_parameters_.inertia_(0, 0),
  //                 &position_controller_.controller_parameters_.inertia_(0, 0));
  // 		GetRosParameter(private_nh_, "inertia/xy",
  //                 position_controller_.controller_parameters_.inertia_(0, 1),
  //                 &position_controller_.controller_parameters_.inertia_(0, 1));
  // 		position_controller_.controller_parameters_.inertia_(1, 0) = position_controller_.controller_parameters_.inertia_(0, 1);
  // 		GetRosParameter(private_nh_, "inertia/xz",
  //                 position_controller_.controller_parameters_.inertia_(0, 2),
  //                 &position_controller_.controller_parameters_.inertia_(0, 2));
  // 		position_controller_.controller_parameters_.inertia_(2, 0) = position_controller_.controller_parameters_.inertia_(0, 2);
  // 		GetRosParameter(private_nh_, "inertia/yy",
  //                 position_controller_.controller_parameters_.inertia_(1, 1),
  //                 &position_controller_.controller_parameters_.inertia_(1, 1));
  // 		GetRosParameter(private_nh_, "inertia/yz",
  //                 position_controller_.controller_parameters_.inertia_(1, 2),
  //                 &position_controller_.controller_parameters_.inertia_(1, 2));
  // 		position_controller_.controller_parameters_.inertia_(2, 1) = position_controller_.controller_parameters_.inertia_(1, 2);
  // 		GetRosParameter(private_nh_, "inertia/zz",
  //                 position_controller_.controller_parameters_.inertia_(2, 2),
  //                 &position_controller_.controller_parameters_.inertia_(2, 2));

		GetRosParameter(private_nh_, "m_cap",
                  position_controller_.controller_parameters_.m_cap_,
                  &position_controller_.controller_parameters_.m_cap_);

		GetRosParameter(private_nh_, "eeta_p/x",
                  position_controller_.controller_parameters_.eeta_p_.x(),
                  &position_controller_.controller_parameters_.eeta_p_.x());
		
		GetRosParameter(private_nh_, "eeta_p/y",
                  position_controller_.controller_parameters_.eeta_p_.y(),
                  &position_controller_.controller_parameters_.eeta_p_.y());

		GetRosParameter(private_nh_, "eeta_p/z",
                  position_controller_.controller_parameters_.eeta_p_.z(),
                  &position_controller_.controller_parameters_.eeta_p_.z());

		GetRosParameter(private_nh_, "eeta_q",
                  position_controller_.controller_parameters_.eeta_q_,
                  &position_controller_.controller_parameters_.eeta_q_);

		GetRosParameter(private_nh_, "length",
                  position_controller_.controller_parameters_.length_,
                  &position_controller_.controller_parameters_.length_);

		GetRosParameter(private_nh_, "arm_length",
                  position_controller_.controller_parameters_.arm_length_,
                  &position_controller_.controller_parameters_.arm_length_);

		GetRosParameter(private_nh_, "side",
                  position_controller_.controller_parameters_.side_,
                  &position_controller_.controller_parameters_.side_);

		GetRosParameter(private_nh_, "gravity",
                  position_controller_.controller_parameters_.gravity_,
                  &position_controller_.controller_parameters_.gravity_);

		GetRosParameter(private_nh_, "payload",
                  position_controller_.controller_parameters_.payload_,
                  &position_controller_.controller_parameters_.payload_);

		GetRosParameter(private_nh_, "rotor_mass",
                  position_controller_.controller_parameters_.rotor_mass_,
                  &position_controller_.controller_parameters_.rotor_mass_);

		GetRosParameter(private_nh_, "var_pi_p",
                  position_controller_.controller_parameters_.var_pi_p_,
                  &position_controller_.controller_parameters_.var_pi_p_);

		GetRosParameter(private_nh_, "var_pi_q",
                  position_controller_.controller_parameters_.var_pi_q_,
                  &position_controller_.controller_parameters_.var_pi_q_);
		
		
		GetVehicleParameters(private_nh_, &position_controller_.vehicle_parameters_);

		position_controller_.InitializeParameters();
	}

	void SmcPositionControllerNode::MdjTrajCallback(
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

	void SmcPositionControllerNode::OdometryCallback(
		const nav_msgs::OdometryConstPtr& odometry_msg) {
		ROS_INFO_ONCE("SmcPositionController got first odometry message.");

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

	void SmcPositionControllerNode::TimedCallback(const ros::TimerEvent& e) {
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

	void SmcPositionControllerNode::PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
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
  ros::init(argc, argv, "smc_position_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rrc_control::SmcPositionControllerNode smc_position_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}