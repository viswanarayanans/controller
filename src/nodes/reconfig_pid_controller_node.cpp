/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <eigen_conversions/eigen_msg.h>

#include "reconfig_pid_controller_node.h"

#include "rrc_control/parameters_ros.h"

namespace rrc_control {

ReconfigPidControllerNode::ReconfigPidControllerNode(
  const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  :nh_(nh),
   private_nh_(private_nh){
  InitializeParams();

  ROS_INFO("Let us initilize the nodes and callbacks");

  cmd_pose_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_POSE, 1,
      &ReconfigPidControllerNode::CommandPoseCallback, this);

  cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &ReconfigPidControllerNode::MultiDofJointTrajectoryCallback, this);

  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &ReconfigPidControllerNode::OdometryCallback, this);

  pose_sub_ = nh_.subscribe("/odometry/pose", 1, 
                            &ReconfigPidControllerNode::PoseCallback, this);

  box_sub_ = nh_.subscribe("/box_data", 1, 
                            &ReconfigPidControllerNode::BoxCallback, this);

  motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  plot_data_pub_ = nh_.advertise<msg_check::PlotDataMsg>("/data_out", 1);

  command_timer_ = nh_.createTimer(ros::Duration(0), &ReconfigPidControllerNode::TimedCommandCallback, this,
                                  true, false);
  // sub = nh_.subscribe("/scan", 1000, &ReconfigPidControllerNode::callback, this);
}

ReconfigPidControllerNode::~ReconfigPidControllerNode() { }

void ReconfigPidControllerNode::InitializeParams() {

  // Read parameters from rosparam.
  GetRosParameter(private_nh_, "position_gain/x",
                  reconfig_pid_controller_.controller_parameters_.position_gain_.x(),
                  &reconfig_pid_controller_.controller_parameters_.position_gain_.x());
  GetRosParameter(private_nh_, "position_gain/y",
                  reconfig_pid_controller_.controller_parameters_.position_gain_.y(),
                  &reconfig_pid_controller_.controller_parameters_.position_gain_.y());
  GetRosParameter(private_nh_, "position_gain/z",
                  reconfig_pid_controller_.controller_parameters_.position_gain_.z(),
                  &reconfig_pid_controller_.controller_parameters_.position_gain_.z());
  GetRosParameter(private_nh_, "velocity_gain/x",
                  reconfig_pid_controller_.controller_parameters_.velocity_gain_.x(),
                  &reconfig_pid_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(private_nh_, "velocity_gain/y",
                  reconfig_pid_controller_.controller_parameters_.velocity_gain_.y(),
                  &reconfig_pid_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(private_nh_, "velocity_gain/z",
                  reconfig_pid_controller_.controller_parameters_.velocity_gain_.z(),
                  &reconfig_pid_controller_.controller_parameters_.velocity_gain_.z());
  GetRosParameter(private_nh_, "position_integral_gain/x",
                  reconfig_pid_controller_.controller_parameters_.position_integral_gain_.x(),
                  &reconfig_pid_controller_.controller_parameters_.position_integral_gain_.x()); //Added by Viswa
  GetRosParameter(private_nh_, "position_integral_gain/y",
                  reconfig_pid_controller_.controller_parameters_.position_integral_gain_.y(),
                  &reconfig_pid_controller_.controller_parameters_.position_integral_gain_.y()); //Added by Viswa
  GetRosParameter(private_nh_, "position_integral_gain/z",
                  reconfig_pid_controller_.controller_parameters_.position_integral_gain_.z(),
                  &reconfig_pid_controller_.controller_parameters_.position_integral_gain_.z()); //Added by Viswa
  GetRosParameter(private_nh_, "attitude_gain/x",
                  reconfig_pid_controller_.controller_parameters_.attitude_gain_.x(),
                  &reconfig_pid_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(private_nh_, "attitude_gain/y",
                  reconfig_pid_controller_.controller_parameters_.attitude_gain_.y(),
                  &reconfig_pid_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(private_nh_, "attitude_gain/z",
                  reconfig_pid_controller_.controller_parameters_.attitude_gain_.z(),
                  &reconfig_pid_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(private_nh_, "angular_rate_gain/x",
                  reconfig_pid_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &reconfig_pid_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(private_nh_, "angular_rate_gain/y",
                  reconfig_pid_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &reconfig_pid_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(private_nh_, "angular_rate_gain/z",
                  reconfig_pid_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &reconfig_pid_controller_.controller_parameters_.angular_rate_gain_.z());
  ROS_INFO_STREAM("Rotor size:" << reconfig_pid_controller_.vehicle_parameters_.rotor_configuration_.rotors.size());
  
  GetRosParameter(private_nh_, "scissors_mass",
                  reconfig_pid_controller_.vehicle_parameters_.scissors_mass_,
                  &reconfig_pid_controller_.vehicle_parameters_.scissors_mass_);
  GetRosParameter(private_nh_, "box_mass",
                  reconfig_pid_controller_.vehicle_parameters_.box_mass_,
                  &reconfig_pid_controller_.vehicle_parameters_.box_mass_);
  GetRosParameter(private_nh_, "rotor_mass",
                  reconfig_pid_controller_.vehicle_parameters_.rotor_mass_,
                  &reconfig_pid_controller_.vehicle_parameters_.rotor_mass_);
  GetRosParameter(private_nh_, "rotor_unit_mass",
                  reconfig_pid_controller_.vehicle_parameters_.rotor_unit_mass_,
                  &reconfig_pid_controller_.vehicle_parameters_.rotor_unit_mass_);
  GetRosParameter(private_nh_, "battery_mass",
                  reconfig_pid_controller_.vehicle_parameters_.battery_mass_,
                  &reconfig_pid_controller_.vehicle_parameters_.battery_mass_);
  GetRosParameter(private_nh_, "rotor_ext_len",
                  reconfig_pid_controller_.vehicle_parameters_.rotor_ext_len_,
                  &reconfig_pid_controller_.vehicle_parameters_.rotor_ext_len_);
  GetRosParameter(private_nh_, "box_dim/x",
                  reconfig_pid_controller_.vehicle_parameters_.box_dim_.x(),
                  &reconfig_pid_controller_.vehicle_parameters_.box_dim_.x());
  GetRosParameter(private_nh_, "box_dim/y",
                  reconfig_pid_controller_.vehicle_parameters_.box_dim_.y(),
                  &reconfig_pid_controller_.vehicle_parameters_.box_dim_.y());
  GetRosParameter(private_nh_, "box_dim/z",
                  reconfig_pid_controller_.vehicle_parameters_.box_dim_.z(),
                  &reconfig_pid_controller_.vehicle_parameters_.box_dim_.z());
  GetRosParameter(private_nh_, "battery_dim/x",
                  reconfig_pid_controller_.vehicle_parameters_.battery_dim_.x(),
                  &reconfig_pid_controller_.vehicle_parameters_.battery_dim_.x());
  GetRosParameter(private_nh_, "battery_dim/y",
                  reconfig_pid_controller_.vehicle_parameters_.battery_dim_.y(),
                  &reconfig_pid_controller_.vehicle_parameters_.battery_dim_.y());
  GetRosParameter(private_nh_, "battery_dim/z",
                  reconfig_pid_controller_.vehicle_parameters_.battery_dim_.z(),
                  &reconfig_pid_controller_.vehicle_parameters_.battery_dim_.z());

  GetRotorConfiguration(private_nh_, &reconfig_pid_controller_.vehicle_parameters_.rotor_configuration_);


  ROS_INFO_STREAM("Rotor size:" << reconfig_pid_controller_.vehicle_parameters_.rotor_configuration_.rotors.size());

  reconfig_pid_controller_.InitializeParameters();
}
void ReconfigPidControllerNode::Publish() {
}

void ReconfigPidControllerNode::BoxCallback(const msg_check::BoxMsgPtr& box_data){
  Eigen::Vector3d dim;
  tf::vectorMsgToEigen(box_data->dimension, dim);
  reconfig_pid_controller_.ResetParameters(box_data->mass, &dim);
}

void ReconfigPidControllerNode::CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  reconfig_pid_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void ReconfigPidControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {

  ROS_INFO_STREAM(" Trajectory message received");

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
  reconfig_pid_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void ReconfigPidControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  reconfig_pid_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void ReconfigPidControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("ReconfigPidController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  reconfig_pid_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  reconfig_pid_controller_.CalculateRotorVelocities(&ref_rotor_velocities, &data_out_);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = odometry_msg->header.stamp;
  data_out_.header.stamp = odometry_msg->header.stamp;


  motor_velocity_reference_pub_.publish(actuator_msg);
  comm_.sendSerial(ref_rotor_velocities);
  plot_data_pub_.publish(data_out_);
}
                                                  
void ReconfigPidControllerNode::PoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
  ROS_INFO_STREAM("ReconfigPidController got first tag message.");

  eigenOdometryFromPoseCovMsg(msg, &odometry);

  reconfig_pid_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  reconfig_pid_controller_.CalculateRotorVelocities(&ref_rotor_velocities, &data_out_);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = msg->header.stamp;
  data_out_.header.stamp = msg->header.stamp;


  motor_velocity_reference_pub_.publish(actuator_msg);
  // comm_.sendSerial(ref_rotor_velocities);
  plot_data_pub_.publish(data_out_);
}

}



int main(int argc, char** argv) {
  ros::init(argc, argv, "reconfig_pid_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rrc_control::ReconfigPidControllerNode reconfig_pid_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}
