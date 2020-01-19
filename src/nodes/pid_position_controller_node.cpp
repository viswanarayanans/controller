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

#include "pid_position_controller_node.h"

#include "viswa_control/parameters_ros.h"

namespace viswa_control {

PidPositionControllerNode::PidPositionControllerNode(
  const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  :nh_(nh),
   private_nh_(private_nh){
  InitializeParams();

  ROS_INFO("Let us initilize the nodes and callbacks");

  cmd_pose_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_POSE, 1,
      &PidPositionControllerNode::CommandPoseCallback, this);

  cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &PidPositionControllerNode::MultiDofJointTrajectoryCallback, this);

  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &PidPositionControllerNode::OdometryCallback, this);

  pose_sub_ = nh_.subscribe("/odometry/pose", 1, 
                            &PidPositionControllerNode::PoseCallback, this);

  motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  command_timer_ = nh_.createTimer(ros::Duration(0), &PidPositionControllerNode::TimedCommandCallback, this,
                                  true, false);
  // sub = nh_.subscribe("/scan", 1000, &PidPositionControllerNode::callback, this);
}

PidPositionControllerNode::~PidPositionControllerNode() { }

void PidPositionControllerNode::InitializeParams() {

  // Read parameters from rosparam.
  GetRosParameter(private_nh_, "position_gain/x",
                  pid_position_controller_.controller_parameters_.position_gain_.x(),
                  &pid_position_controller_.controller_parameters_.position_gain_.x());
  GetRosParameter(private_nh_, "position_gain/y",
                  pid_position_controller_.controller_parameters_.position_gain_.y(),
                  &pid_position_controller_.controller_parameters_.position_gain_.y());
  GetRosParameter(private_nh_, "position_gain/z",
                  pid_position_controller_.controller_parameters_.position_gain_.z(),
                  &pid_position_controller_.controller_parameters_.position_gain_.z());
  GetRosParameter(private_nh_, "velocity_gain/x",
                  pid_position_controller_.controller_parameters_.velocity_gain_.x(),
                  &pid_position_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(private_nh_, "velocity_gain/y",
                  pid_position_controller_.controller_parameters_.velocity_gain_.y(),
                  &pid_position_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(private_nh_, "velocity_gain/z",
                  pid_position_controller_.controller_parameters_.velocity_gain_.z(),
                  &pid_position_controller_.controller_parameters_.velocity_gain_.z());
  GetRosParameter(private_nh_, "position_integral_gain/x",
                  pid_position_controller_.controller_parameters_.position_integral_gain_.x(),
                  &pid_position_controller_.controller_parameters_.position_integral_gain_.x()); //Added by Viswa
  GetRosParameter(private_nh_, "position_integral_gain/y",
                  pid_position_controller_.controller_parameters_.position_integral_gain_.y(),
                  &pid_position_controller_.controller_parameters_.position_integral_gain_.y()); //Added by Viswa
  GetRosParameter(private_nh_, "position_integral_gain/z",
                  pid_position_controller_.controller_parameters_.position_integral_gain_.z(),
                  &pid_position_controller_.controller_parameters_.position_integral_gain_.z()); //Added by Viswa
  GetRosParameter(private_nh_, "attitude_gain/x",
                  pid_position_controller_.controller_parameters_.attitude_gain_.x(),
                  &pid_position_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(private_nh_, "attitude_gain/y",
                  pid_position_controller_.controller_parameters_.attitude_gain_.y(),
                  &pid_position_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(private_nh_, "attitude_gain/z",
                  pid_position_controller_.controller_parameters_.attitude_gain_.z(),
                  &pid_position_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(private_nh_, "angular_rate_gain/x",
                  pid_position_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &pid_position_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(private_nh_, "angular_rate_gain/y",
                  pid_position_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &pid_position_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(private_nh_, "angular_rate_gain/z",
                  pid_position_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &pid_position_controller_.controller_parameters_.angular_rate_gain_.z());
  ROS_INFO_STREAM("Rotor size:" << pid_position_controller_.vehicle_parameters_.rotor_configuration_.rotors.size());
  GetVehicleParameters(private_nh_, &pid_position_controller_.vehicle_parameters_);
  ROS_INFO_STREAM("Rotor size:" << pid_position_controller_.vehicle_parameters_.rotor_configuration_.rotors.size());

  pid_position_controller_.InitializeParameters();
}
void PidPositionControllerNode::Publish() {
}

void PidPositionControllerNode::CommandPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  pid_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void PidPositionControllerNode::MultiDofJointTrajectoryCallback(
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
  pid_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void PidPositionControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  pid_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void PidPositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("PidPositionController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  pid_position_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  pid_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(actuator_msg);
}
                                                  
void PidPositionControllerNode::PoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
  ROS_INFO_STREAM("PidPositionController got first tag message.");

  eigenOdometryFromPoseMsg(msg, &odometry);

  pid_position_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  pid_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = msg->header.stamp;

  motor_velocity_reference_pub_.publish(actuator_msg);
}

}



int main(int argc, char** argv) {
  ros::init(argc, argv, "pid_position_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  viswa_control::PidPositionControllerNode pid_position_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}
