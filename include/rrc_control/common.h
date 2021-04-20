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

#ifndef INCLUDE_RRC_CONTROL_COMMON_H_
#define INCLUDE_RRC_CONTROL_COMMON_H_

#include <assert.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


#include "rrc_control/parameters.h"

namespace rrc_control {

// Default values.
static const std::string kDefaultNamespace = "";
static const std::string kDefaultCommandMotorSpeedTopic =
    mav_msgs::default_topics::COMMAND_ACTUATORS; // "command/motor_speed";
static const std::string kDefaultCommandMultiDofJointTrajectoryTopic =
    mav_msgs::default_topics::COMMAND_TRAJECTORY; // "command/trajectory"
static const std::string kDefaultCommandRollPitchYawrateThrustTopic =
    mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST;
    // "command/roll_pitch_yawrate_thrust"
static const std::string kDefaultImuTopic =
    mav_msgs::default_topics::IMU; // "imu
static const std::string kDefaultOdometryTopic =
    mav_msgs::default_topics::ODOMETRY; // "odometry"

struct EigenOdometry {
  EigenOdometry()
      : position(0.0, 0.0, 0.0),
        orientation(Eigen::Quaterniond::Identity()),
        velocity(0.0, 0.0, 0.0),
        angular_velocity(0.0, 0.0, 0.0),
        timestamp_ns(0) {};

  EigenOdometry(const Eigen::Vector3d& _position,
                const Eigen::Quaterniond& _orientation,
                const Eigen::Vector3d& _velocity,
                const Eigen::Vector3d& _angular_velocity,
                const uint64_t _timestamp_ns) {
    position = _position;
    orientation = _orientation;
    velocity = _velocity;
    angular_velocity = _angular_velocity;
    timestamp_ns = _timestamp_ns;
  };

  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d velocity; // Velocity is expressed in the Body frame!
  Eigen::Vector3d angular_velocity;
  uint64_t timestamp_ns;
};

// struct PlotData {
//   PlotData()
//     : sp(0.0, 0.0, 0.0),
//       sq(0.0, 0.0, 0.0),
//       xq(0.0, 0.0, 0.0),
//       position_error(0.0, 0.0, 0.0),
//       velocity_error(0.0, 0.0, 0.0),
//       angle_error(0.0, 0.0, 0.0),
//       angle_rate_error(0.0, 0.0, 0.0),
//       Kp_hat(0.001, 0.001, 0.001),
//       Kq_hat_0(0.0001, 0.0001, 0.0001),
//       Kq_hat_1(0.0001, 0.0001, 0.0001),
//       Kq_hat_2(0.0001, 0.0001, 0.0001),
//       rho_p(0.0, 0.0, 0.0),
//       rho_q(0.0, 0.0, 0.0),
//       delTau_p(0.0, 0.0, 0.0),
//       delTau_q(0.0, 0.0, 0.0),
//       M_hat(0.1),
//       timestamp_ns(0) {};

//   Eigen::Vector3d sp;
//   Eigen::Vector3d sq;

//   // Eigen::Vector3d xp;
//   Eigen::Vector3d xq;

//   Eigen::Vector3d acceleration;
//   Eigen::Vector3d angular_acceleration;

//   Eigen::Vector3d position_error;
//   Eigen::Vector3d velocity_error;
//   Eigen::Vector3d position_error_integral;
//   // Eigen::Vector3d acceleration_error;
//   Eigen::Vector3d angle_error;
//   Eigen::Vector3d angle_rate_error;
//   // Eigen::Vector3d angle_acc_error;

//   Eigen::Vector3d Kp_hat;
//   Eigen::Vector3d Kq_hat_0;
//   Eigen::Vector3d Kq_hat_1;
//   Eigen::Vector3d Kq_hat_2;

//   Eigen::Vector3d rho_p;
//   Eigen::Vector3d rho_q;
//   Eigen::Vector3d delTau_p;
//   Eigen::Vector3d delTau_q;

//   double thrust;
//   Eigen::Vector3d moments;


//   double M_hat;
//   uint64_t timestamp_ns;
// };


inline void eigenOdometryFromMsg(const nav_msgs::OdometryConstPtr& msg,
                                 EigenOdometry* odometry) {
  odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.pose.position);
  odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);
  odometry->velocity = mav_msgs::vector3FromMsg(msg->twist.twist.linear);
  odometry->angular_velocity = mav_msgs::vector3FromMsg(msg->twist.twist.angular);
}

inline void calculateAllocationMatrix(const RotorConfiguration& rotor_configuration,
                                      Eigen::Matrix4Xd* allocation_matrix) {
  assert(allocation_matrix != nullptr);
  allocation_matrix->resize(4, rotor_configuration.rotors.size());
  unsigned int i = 0;
  for (const Rotor& rotor : rotor_configuration.rotors) {
    // Set first row of allocation matrix.
    (*allocation_matrix)(0, i) = sin(rotor.angle) * rotor.arm_length
                                 * rotor.rotor_force_constant;
    // Set second row of allocation matrix.
    (*allocation_matrix)(1, i) = -cos(rotor.angle) * rotor.arm_length
                                 * rotor.rotor_force_constant;
    // Set third row of allocation matrix.
    (*allocation_matrix)(2, i) = -rotor.direction * rotor.rotor_force_constant
                                 * rotor.rotor_moment_constant;
    // Set forth row of allocation matrix.
    (*allocation_matrix)(3, i) = rotor.rotor_force_constant;
    ++i;
  }
  Eigen::FullPivLU<Eigen::Matrix4Xd> lu(*allocation_matrix);
  // Setting the threshold for when pivots of the rank calculation should be considered nonzero.
  lu.setThreshold(1e-9);
  int rank = lu.rank();
  if (rank < 4) {
    std::cout << "The rank of the allocation matrix is " << lu.rank()
              << ", it should have rank 4, to have a fully controllable system,"
              << " check your configuration." << std::endl;
  }

}

inline void skewMatrixFromVector(Eigen::Vector3d& vector, Eigen::Matrix3d* skew_matrix) {
  *skew_matrix << 0, -vector.z(), vector.y(),
                  vector.z(), 0, -vector.x(),
                  -vector.y(), vector.x(), 0;
}

inline void vectorFromSkewMatrix(Eigen::Matrix3d& skew_matrix, Eigen::Vector3d* vector) {
  *vector << skew_matrix(2, 1), skew_matrix(0,2), skew_matrix(1, 0);
}


inline void eigenOdometryFromPoseMsg(const geometry_msgs::PoseStamped::ConstPtr& msg,
                                 EigenOdometry* odometry) {
  assert(odometry);

  double dt = (double)(msg->header.stamp.toNSec() - odometry->timestamp_ns) * 1e-9;
  if (dt > 0.05){
    dt = 0.05;
  }
  else if (dt < 0.01){
    dt = 0.01;
  }

  odometry->velocity = (mav_msgs::vector3FromPointMsg(msg->pose.position) - 
                          odometry->position) / dt;
  Eigen::Matrix3d R_cur = (mav_msgs::quaternionFromMsg(msg->pose.orientation)).toRotationMatrix(); 
  Eigen::Matrix3d R_pre = (odometry->orientation).toRotationMatrix();

  Eigen::Matrix3d R_diff =  0.5 * (R_pre.transpose() * R_cur - R_cur.transpose() * R_pre);
  Eigen::Vector3d angle_diff;
  vectorFromSkewMatrix(R_diff, &angle_diff);
  odometry->angular_velocity = angle_diff / dt;

  odometry->timestamp_ns = msg->header.stamp.toNSec();
  odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.position);
  odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.orientation);

  // odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.position);
  // odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);
  // odometry->velocity = mav_msgs::vector3FromMsg(msg->twist.twist.linear);
  // odometry->angular_velocity = mav_msgs::vector3FromMsg(msg->twist.twist.angular);
}

inline void eigenOdometryFromPoseCovMsg(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg,
                                 EigenOdometry* odometry) {
  assert(odometry);

  double dt = (double)(msg->header.stamp.toNSec() - odometry->timestamp_ns) * 1e-9;

  odometry->velocity = (mav_msgs::vector3FromPointMsg(msg->pose.pose.position) - 
                          odometry->position) / dt;
  Eigen::Matrix3d R_cur = (mav_msgs::quaternionFromMsg(msg->pose.pose.orientation)).toRotationMatrix(); 
  Eigen::Matrix3d R_pre = (odometry->orientation).toRotationMatrix();

  Eigen::Matrix3d R_diff =  0.5 * (R_pre.transpose() * R_cur - R_cur.transpose() * R_pre);
  Eigen::Vector3d angle_diff;
  vectorFromSkewMatrix(R_diff, &angle_diff);
  odometry->angular_velocity = angle_diff / dt;

  odometry->timestamp_ns = msg->header.stamp.toNSec();
  odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.pose.position);
  odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);

  // odometry->position = mav_msgs::vector3FromPointMsg(msg->pose.position);
  // odometry->orientation = mav_msgs::quaternionFromMsg(msg->pose.pose.orientation);
  // odometry->velocity = mav_msgs::vector3FromMsg(msg->twist.twist.linear);
  // odometry->angular_velocity = mav_msgs::vector3FromMsg(msg->twist.twist.angular);
}

}

#endif /* INCLUDE_ASMC_CONTROL_COMMON_H_ */
