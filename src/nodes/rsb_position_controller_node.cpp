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

#include "rsb_position_controller_node.h"

#include "rrc_control/parameters_ros.h"

namespace rrc_control {

RsbPositionControllerNode::RsbPositionControllerNode(
  const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  :nh_(nh),
   private_nh_(private_nh){
  InitializeParams();

  ROS_INFO("Let us initilize the nodes and callbacks");

  cmd_pose_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_POSE, 1,
      &RsbPositionControllerNode::CommandPoseCallback, this);

  cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &RsbPositionControllerNode::MultiDofJointTrajectoryCallback, this);

  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &RsbPositionControllerNode::OdometryCallback, this);

  pose_sub_ = nh_.subscribe("/odometry/pose", 1, 
                            &RsbPositionControllerNode::PoseCallback, this);

  motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  plot_data_pub_ = nh_.advertise<msg_check::PlotDataMsg>("/data_out", 1);

  command_timer_ = nh_.createTimer(ros::Duration(0), &RsbPositionControllerNode::TimedCommandCallback, this,
                                  true, false);
  // sub = nh_.subscribe("/scan", 1000, &RsbPositionControllerNode::callback, this);
}

RsbPositionControllerNode::~RsbPositionControllerNode() { }

void RsbPositionControllerNode::InitializeParams() {

  // Read parameters from rosparam.
  GetRosParameter(private_nh_, "kbp/x",
                  rsb_position_controller_.controller_parameters_.kbp_.x(),
                  &rsb_position_controller_.controller_parameters_.kbp_.x());
  GetRosParameter(private_nh_, "kbp/y",
                  rsb_position_controller_.controller_parameters_.kbp_.y(),
                  &rsb_position_controller_.controller_parameters_.kbp_.y());
  GetRosParameter(private_nh_, "kbp/z",
                  rsb_position_controller_.controller_parameters_.kbp_.z(),
                  &rsb_position_controller_.controller_parameters_.kbp_.z());
  GetRosParameter(private_nh_, "K1p/x",
                  rsb_position_controller_.controller_parameters_.K1p_.x(),
                  &rsb_position_controller_.controller_parameters_.K1p_.x());
  GetRosParameter(private_nh_, "K1p/y",
                  rsb_position_controller_.controller_parameters_.K1p_.y(),
                  &rsb_position_controller_.controller_parameters_.K1p_.y());
  GetRosParameter(private_nh_, "K1p/z",
                  rsb_position_controller_.controller_parameters_.K1p_.z(),
                  &rsb_position_controller_.controller_parameters_.K1p_.z());
  GetRosParameter(private_nh_, "K2p/x",
                  rsb_position_controller_.controller_parameters_.K2p_.x(),
                  &rsb_position_controller_.controller_parameters_.K2p_.x()); 
  GetRosParameter(private_nh_, "K2p/y",
                  rsb_position_controller_.controller_parameters_.K2p_.y(),
                  &rsb_position_controller_.controller_parameters_.K2p_.y()); 
  GetRosParameter(private_nh_, "K2p/z",
                  rsb_position_controller_.controller_parameters_.K2p_.z(),
                  &rsb_position_controller_.controller_parameters_.K2p_.z()); 
  GetRosParameter(private_nh_, "kbq/x",
                  rsb_position_controller_.controller_parameters_.kbq_.x(),
                  &rsb_position_controller_.controller_parameters_.kbq_.x());
  GetRosParameter(private_nh_, "kbq/y",
                  rsb_position_controller_.controller_parameters_.kbq_.y(),
                  &rsb_position_controller_.controller_parameters_.kbq_.y());
  GetRosParameter(private_nh_, "kbq/z",
                  rsb_position_controller_.controller_parameters_.kbq_.z(),
                  &rsb_position_controller_.controller_parameters_.kbq_.z());
  GetRosParameter(private_nh_, "K1q/x",
                  rsb_position_controller_.controller_parameters_.K1q_.x(),
                  &rsb_position_controller_.controller_parameters_.K1q_.x());
  GetRosParameter(private_nh_, "K1q/y",
                  rsb_position_controller_.controller_parameters_.K1q_.y(),
                  &rsb_position_controller_.controller_parameters_.K1q_.y());
  GetRosParameter(private_nh_, "K1q/z",
                  rsb_position_controller_.controller_parameters_.K1q_.z(),
                  &rsb_position_controller_.controller_parameters_.K1q_.z());
  GetRosParameter(private_nh_, "K2q/x",
                  rsb_position_controller_.controller_parameters_.K2q_.x(),
                  &rsb_position_controller_.controller_parameters_.K2q_.x()); 
  GetRosParameter(private_nh_, "K2q/y",
                  rsb_position_controller_.controller_parameters_.K2q_.y(),
                  &rsb_position_controller_.controller_parameters_.K2q_.y()); 
  GetRosParameter(private_nh_, "K2q/z",
                  rsb_position_controller_.controller_parameters_.K2q_.z(),
                  &rsb_position_controller_.controller_parameters_.K2q_.z());
                   
  GetRosParameter(private_nh_, "sigma_q/x",
                  rsb_position_controller_.controller_parameters_.sigma_q_.x(),
                  &rsb_position_controller_.controller_parameters_.sigma_q_.x()); 
  GetRosParameter(private_nh_, "sigma_q/y",
                  rsb_position_controller_.controller_parameters_.sigma_q_.y(),
                  &rsb_position_controller_.controller_parameters_.sigma_q_.y()); 
  GetRosParameter(private_nh_, "sigma_q/z",
                  rsb_position_controller_.controller_parameters_.sigma_q_.z(),
                  &rsb_position_controller_.controller_parameters_.sigma_q_.z()); 
  GetRosParameter(private_nh_, "sigma_p/x",
                  rsb_position_controller_.controller_parameters_.sigma_p_.x(),
                  &rsb_position_controller_.controller_parameters_.sigma_p_.x()); 
  GetRosParameter(private_nh_, "sigma_p/y",
                  rsb_position_controller_.controller_parameters_.sigma_p_.y(),
                  &rsb_position_controller_.controller_parameters_.sigma_p_.y()); 
  GetRosParameter(private_nh_, "sigma_p/z",
                  rsb_position_controller_.controller_parameters_.sigma_p_.z(),
                  &rsb_position_controller_.controller_parameters_.sigma_p_.z());

  GetRosParameter(private_nh_, "E_hat_q/x",
                  rsb_position_controller_.controller_parameters_.E_hat_q_.x(),
                  &rsb_position_controller_.controller_parameters_.E_hat_q_.x()); 
  GetRosParameter(private_nh_, "E_hat_q/y",
                  rsb_position_controller_.controller_parameters_.E_hat_q_.y(),
                  &rsb_position_controller_.controller_parameters_.E_hat_q_.y()); 
  GetRosParameter(private_nh_, "E_hat_q/z",
                  rsb_position_controller_.controller_parameters_.E_hat_q_.z(),
                  &rsb_position_controller_.controller_parameters_.E_hat_q_.z()); 
  GetRosParameter(private_nh_, "E_hat_p/x",
                  rsb_position_controller_.controller_parameters_.E_hat_p_.x(),
                  &rsb_position_controller_.controller_parameters_.E_hat_p_.x()); 
  GetRosParameter(private_nh_, "E_hat_p/y",
                  rsb_position_controller_.controller_parameters_.E_hat_p_.y(),
                  &rsb_position_controller_.controller_parameters_.E_hat_p_.y()); 
  GetRosParameter(private_nh_, "E_hat_p/z",
                  rsb_position_controller_.controller_parameters_.E_hat_p_.z(),
                  &rsb_position_controller_.controller_parameters_.E_hat_p_.z());

  GetRosParameter(private_nh_, "c1_q/x",
                  rsb_position_controller_.controller_parameters_.c1_q_.x(),
                  &rsb_position_controller_.controller_parameters_.c1_q_.x()); 
  GetRosParameter(private_nh_, "c1_q/y",
                  rsb_position_controller_.controller_parameters_.c1_q_.y(),
                  &rsb_position_controller_.controller_parameters_.c1_q_.y()); 
  GetRosParameter(private_nh_, "c1_q/z",
                  rsb_position_controller_.controller_parameters_.c1_q_.z(),
                  &rsb_position_controller_.controller_parameters_.c1_q_.z()); 
  GetRosParameter(private_nh_, "c1_p/x",
                  rsb_position_controller_.controller_parameters_.c1_p_.x(),
                  &rsb_position_controller_.controller_parameters_.c1_p_.x()); 
  GetRosParameter(private_nh_, "c1_p/y",
                  rsb_position_controller_.controller_parameters_.c1_p_.y(),
                  &rsb_position_controller_.controller_parameters_.c1_p_.y()); 
  GetRosParameter(private_nh_, "c1_p/z",
                  rsb_position_controller_.controller_parameters_.c1_p_.z(),
                  &rsb_position_controller_.controller_parameters_.c1_p_.z());

  GetRosParameter(private_nh_, "c2_q/x",
                  rsb_position_controller_.controller_parameters_.c2_q_.x(),
                  &rsb_position_controller_.controller_parameters_.c2_q_.x()); 
  GetRosParameter(private_nh_, "c2_q/y",
                  rsb_position_controller_.controller_parameters_.c2_q_.y(),
                  &rsb_position_controller_.controller_parameters_.c2_q_.y()); 
  GetRosParameter(private_nh_, "c2_q/z",
                  rsb_position_controller_.controller_parameters_.c2_q_.z(),
                  &rsb_position_controller_.controller_parameters_.c2_q_.z()); 
  GetRosParameter(private_nh_, "c2_p/x",
                  rsb_position_controller_.controller_parameters_.c2_p_.x(),
                  &rsb_position_controller_.controller_parameters_.c2_p_.x()); 
  GetRosParameter(private_nh_, "c2_p/y",
                  rsb_position_controller_.controller_parameters_.c2_p_.y(),
                  &rsb_position_controller_.controller_parameters_.c2_p_.y()); 
  GetRosParameter(private_nh_, "c2_p/z",
                  rsb_position_controller_.controller_parameters_.c2_p_.z(),
                  &rsb_position_controller_.controller_parameters_.c2_p_.z());

  GetRosParameter(private_nh_, "eeta_q/x",
                  rsb_position_controller_.controller_parameters_.eeta_q_.x(),
                  &rsb_position_controller_.controller_parameters_.eeta_q_.x()); 
  GetRosParameter(private_nh_, "eeta_q/y",
                  rsb_position_controller_.controller_parameters_.eeta_q_.y(),
                  &rsb_position_controller_.controller_parameters_.eeta_q_.y()); 
  GetRosParameter(private_nh_, "eeta_q/z",
                  rsb_position_controller_.controller_parameters_.eeta_q_.z(),
                  &rsb_position_controller_.controller_parameters_.eeta_q_.z()); 
  GetRosParameter(private_nh_, "eeta_p/x",
                  rsb_position_controller_.controller_parameters_.eeta_p_.x(),
                  &rsb_position_controller_.controller_parameters_.eeta_p_.x()); 
  GetRosParameter(private_nh_, "eeta_p/y",
                  rsb_position_controller_.controller_parameters_.eeta_p_.y(),
                  &rsb_position_controller_.controller_parameters_.eeta_p_.y()); 
  GetRosParameter(private_nh_, "eeta_p/z",
                  rsb_position_controller_.controller_parameters_.eeta_p_.z(),
                  &rsb_position_controller_.controller_parameters_.eeta_p_.z()); 
  
  ROS_INFO_STREAM("Rotor size:" << rsb_position_controller_.vehicle_parameters_.rotor_configuration_.rotors.size());
  GetVehicleParameters(private_nh_, &rsb_position_controller_.vehicle_parameters_);
  ROS_INFO_STREAM("Rotor size:" << rsb_position_controller_.vehicle_parameters_.rotor_configuration_.rotors.size());

  rsb_position_controller_.InitializeParameters();
}
void RsbPositionControllerNode::Publish() {
}

void RsbPositionControllerNode::CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  rsb_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void RsbPositionControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {

  // ROS_INFO_STREAM(" Trajectory message received");

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
  rsb_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void RsbPositionControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  rsb_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void RsbPositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("RsbPositionController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  rsb_position_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  rsb_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities, &data_out_);

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
                                                  
void RsbPositionControllerNode::PoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
  ROS_INFO_STREAM("RsbPositionController got first tag message.");

  eigenOdometryFromPoseCovMsg(msg, &odometry);

  rsb_position_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  rsb_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities, &data_out_);

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
  ros::init(argc, argv, "rsb_position_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rrc_control::RsbPositionControllerNode rsb_position_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}
