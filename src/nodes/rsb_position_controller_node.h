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

#ifndef RRC_CONTROL_PID_POSITION_CONTROLLER_NODE_H
#define RRC_CONTROL_PID_POSITION_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
//#include <apriltag_ros/AprilTagDetectionArray.h>

#include "rrc_control/common.h"
#include "rrc_control/rsb_position_controller.h"
#include "msg_check/PlotDataMsg.h"
#include <serial_comm.h>


namespace rrc_control {

class RsbPositionControllerNode {
 public:
  RsbPositionControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  ~RsbPositionControllerNode();

  void InitializeParams();
  void Publish();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::NodeHandle nhs_;

  RsbPositionController rsb_position_controller_;
  msg_check::PlotDataMsg data_out_;
  SerialComm comm_;

  std::string namespace_;

  // subscribers
  ros::Subscriber cmd_trajectory_sub_;
  ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
  ros::Subscriber cmd_pose_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber pose_sub_;

  ros::Publisher motor_velocity_reference_pub_;
  ros::Publisher plot_data_pub_;

  mav_msgs::EigenTrajectoryPointDeque commands_;
  std::deque<ros::Duration> command_waiting_times_;
  ros::Timer command_timer_;

  EigenOdometry odometry;

  void TimedCommandCallback(const ros::TimerEvent& e);

  void MultiDofJointTrajectoryCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);

  void CommandPoseCallback(
      const geometry_msgs::PoseStampedConstPtr& pose_msg);

  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);

  void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
   //void TagCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

};
}


#endif // RRC_CONTROL_RSB_POSITION_CONTROLLER_NODE_H
