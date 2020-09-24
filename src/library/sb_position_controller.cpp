/*  * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland  * Copyright
2015 Michael Burri, ASL, ETH Zurich, Switzerland  * Copyright 2015 Mina Kamel,
ASL, ETH Zurich, Switzerland  * Copyright 2015 Janosch Nikolic, ASL, ETH
Zurich, Switzerland  * Copyright 2015 Markus Achtelik, ASL, ETH Zurich,
Switzerland  *  * Licensed under the Apache License, Version 2.0 (the
"License");  * you may not use this file except in compliance with the
License.  * You may obtain a copy of the License at  *  *
http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rrc_control/sb_position_controller.h"
#include <eigen_conversions/eigen_msg.h>

namespace rrc_control {

SbPositionController::SbPositionController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

SbPositionController::~SbPositionController() {}

void SbPositionController::InitializeParameters() {
  
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  rotor_vel_coef_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  rotor_vel_coef_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse();

  // To make the tuning independent of the inertia matrix we divide here.
  // normalized_attitude_gain_ = controller_parameters_.attitude_gain_;
  // // To make the tuning independent of the inertia matrix we divide here.
  // normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_;

  //Added by Viswa : To add payload mass with base mass
  normalized_mass = vehicle_parameters_.mass_ + vehicle_parameters_.payload_mass_;

  initialized_params_ = true;
}

void SbPositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities, 
      msg_check::PlotDataMsg* data_out) const {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // ROS_INFO_STREAM("Rotor velocities calculated: " << rotor_velocities->size());
  
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }
  Eigen::Vector3d thrust_3d;
  CalculateThrust(&thrust_3d, data_out);
  // ROS_INFO("Thrust done");

  double thrust = thrust_3d.dot(odometry_.orientation.toRotationMatrix().col(2)); //Added by Viswa
  // ROS_INFO_STREAM(thrust);
  data_out->thrust = thrust;


  Eigen::Vector3d moments;
  CalculateMoments(thrust_3d, &moments, data_out);
  tf::vectorEigenToMsg(moments, data_out->moments);


  // Project thrust onto body z axis.

  Eigen::Vector4d moment_thrust;
  moment_thrust.block<3, 1>(0, 0) = moments;
  moment_thrust(3) = thrust;

  *rotor_velocities = rotor_vel_coef_ * moment_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}

void SbPositionController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void SbPositionController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void SbPositionController::CalculateThrust(Eigen::Vector3d* thrust, 
  msg_check::PlotDataMsg* data_out) const {
  assert(thrust);

  Eigen::Vector3d position_error;
  position_error = odometry_.position - command_trajectory_.position_W;

  // Transform velocity to world frame.
  const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
  Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
  Eigen::Vector3d velocity_error;
  velocity_error = velocity_W - command_trajectory_.velocity_W;

  Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

  static ros::Time last_time = ros::Time::now();
  ros::Time current_time = ros::Time::now(); 
  double dt = (current_time - last_time).toSec();

  
  Eigen::Vector3d sqr_term_p = controller_parameters_.kbp_.cwiseProduct(controller_parameters_.kbp_) - position_error.cwiseProduct(position_error);
  // static Eigen::Vector3d previous_alpha(0,0,0);

  Eigen::Vector3d alpha_p = -sqr_term_p.cwiseProduct(controller_parameters_.K1p_.cwiseProduct(position_error)) + command_trajectory_.velocity_W;

  Eigen::Vector3d z2_p = velocity_W - alpha_p;
  Eigen::Vector3d dot_alpha_p;


  // dot_alpha_p = (alpha - previous_alpha)/0.01;
  dot_alpha_p = 2*position_error.cwiseProduct(velocity_error).cwiseProduct(controller_parameters_.K1p_).cwiseProduct(position_error)
                - sqr_term_p.cwiseProduct(controller_parameters_.K1p_.cwiseProduct(velocity_error))
                + command_trajectory_.acceleration_W;
  // dot_alpha_p<< 0, 0, 0;

  Eigen::Vector3d term_3_p(0,0,0);
  term_3_p << position_error[0]/sqr_term_p[0], position_error[1]/sqr_term_p[1], position_error[2]/sqr_term_p[2];
  // ROS_INFO_STREAM("position error" << position_error[0] << position_error[1] << position_error[2]);

  *thrust = vehicle_parameters_.mass_*(vehicle_parameters_.gravity_ * e_3 + dot_alpha_p
            - controller_parameters_.K2p_.cwiseProduct(z2_p) - term_3_p);

  last_time = current_time;
  // previous_alpha_p = alpha_p;
  

  tf::vectorEigenToMsg(position_error, data_out->position_error);
  tf::vectorEigenToMsg(velocity_error, data_out->velocity_error);
}

// Implementation from the T. Sb et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void SbPositionController::CalculateMoments(Eigen::Vector3d force, 
                        Eigen::Vector3d* moments, 
                        msg_check::PlotDataMsg* data_out) const {
  assert(moments);
  // ROS_INFO_STREAM("force" << force); 
  if (force[2] >= DBL_MAX || force[2] <= -DBL_MAX) {
    *moments << 0, 0, 0;
  }

  else {

    Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();

    // Get the desired rotation matrix.
    Eigen::Vector3d b1_des;
    double yaw = command_trajectory_.getYaw();
    b1_des << cos(yaw), sin(yaw), 0;

    Eigen::Vector3d b3_des;
    b3_des = force / force.norm();

    Eigen::Vector3d b2_des;
    b2_des = b3_des.cross(b1_des);
    b2_des.normalize();

    Eigen::Matrix3d R_des;
    R_des.col(0) = b2_des.cross(b3_des);
    R_des.col(1) = b2_des;
    R_des.col(2) = b3_des;

    // Angle error according to sb et al.
    Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
    Eigen::Vector3d angle_error;
    vectorFromSkewMatrix(angle_error_matrix, &angle_error);

    // TODO(burrimi) include angular rate references at some point.
    Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
    angular_rate_des[2] = command_trajectory_.getYawRate();
    Eigen:: Vector3d angular_rate_des_B = R_des.transpose() * R * angular_rate_des;

    Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - angular_rate_des_B;

    Eigen::Vector3d sqr_term_q = controller_parameters_.kbq_.cwiseProduct(controller_parameters_.kbq_)
                        - angle_error.cwiseProduct(angle_error);

    Eigen::Vector3d alpha_q = -sqr_term_q.cwiseProduct(controller_parameters_.K1q_.cwiseProduct(angle_error)) 
                              + angular_rate_des_B;

    Eigen::Vector3d dot_alpha_q = 2*angle_error.cwiseProduct(angular_rate_error).cwiseProduct(controller_parameters_.K1q_.cwiseProduct(angle_error))
                                  - sqr_term_q.cwiseProduct(controller_parameters_.K1q_.cwiseProduct(angular_rate_error));

    Eigen::Vector3d z2_q = odometry_.angular_velocity - alpha_q;

    Eigen::Vector3d term_3_q(0,0,0);
    term_3_q << angle_error[0]/sqr_term_q[0], angle_error[1]/sqr_term_q[1], angle_error[2]/sqr_term_q[2];

    // *moments = - angle_error.cwiseProduct(normalized_attitude_gain_)
    //            - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
    //            + odometry_.angular_velocity.cross(vehicle_parameters_.inertia_*odometry_.angular_velocity);

    *moments = vehicle_parameters_.inertia_*(dot_alpha_q 
              - (controller_parameters_.K2q_.cwiseProduct(z2_q)) - term_3_q);
    

    // ROS_INFO_STREAM(*moments);
    // data_out->angular_acceleration = *angular_acceleration;
    // data_out->angle_error = angle_error;
    // data_out->angle_rate_error = angular_rate_error;
    // tf::vectorEigenToMsg(*angular_acceleration, data_out->angular_acceleration);
    tf::vectorEigenToMsg(angle_error, data_out->angle_error);
    tf::vectorEigenToMsg(angular_rate_error, data_out->angle_rate_error);
  }
}

void SbPositionController::CalculateMoments_combined(Eigen::Vector3d force, 
                        Eigen::Vector3d* moments, 
                        msg_check::PlotDataMsg* data_out) const {
  assert(moments);
  // ROS_INFO_STREAM("force" << force); 
  if (force[2] >= DBL_MAX || force[2] <= -DBL_MAX) {
    *moments << 0, 0, 0;
  }

  else {

    Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();

    // Get the desired rotation matrix.
    Eigen::Vector3d b1_des;
    double yaw = command_trajectory_.getYaw();
    b1_des << cos(yaw), sin(yaw), 0;

    Eigen::Vector3d b3_des;
    b3_des = force / force.norm();

    Eigen::Vector3d b2_des;
    b2_des = b3_des.cross(b1_des);
    b2_des.normalize();

    Eigen::Matrix3d R_des;
    R_des.col(0) = b2_des.cross(b3_des);
    R_des.col(1) = b2_des;
    R_des.col(2) = b3_des;

    // Angle error according to sb et al.
    Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
    Eigen::Vector3d angle_error;
    vectorFromSkewMatrix(angle_error_matrix, &angle_error);

    // TODO(burrimi) include angular rate references at some point.
    Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
    angular_rate_des[2] = command_trajectory_.getYawRate();
    Eigen:: Vector3d angular_rate_des_B = R_des.transpose() * R * angular_rate_des;

    Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - angular_rate_des_B;

    double sqr_term_q = controller_parameters_.kbq_.dot(controller_parameters_.kbq_)
                        - angle_error.dot(angle_error);

    Eigen::Vector3d alpha_q = -sqr_term_q*(controller_parameters_.K1q_.cwiseProduct(angle_error)) 
                              + angular_rate_des_B;

    Eigen::Vector3d dot_alpha_q = 2 * (angle_error.dot(angular_rate_error))
                                  * controller_parameters_.K1q_.cwiseProduct(angle_error)
                                  - sqr_term_q*controller_parameters_.K1q_.cwiseProduct(angular_rate_error);

    Eigen::Vector3d z2_q = odometry_.angular_velocity - alpha_q;

    // *moments = - angle_error.cwiseProduct(normalized_attitude_gain_)
    //            - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
    //            + odometry_.angular_velocity.cross(vehicle_parameters_.inertia_*odometry_.angular_velocity);

    *moments = vehicle_parameters_.inertia_*(dot_alpha_q 
              - (controller_parameters_.K2q_.cwiseProduct(z2_q)) - angle_error/sqr_term_q);
    

    // ROS_INFO_STREAM(*moments);
    // data_out->angular_acceleration = *angular_acceleration;
    // data_out->angle_error = angle_error;
    // data_out->angle_rate_error = angular_rate_error;
    // tf::vectorEigenToMsg(*angular_acceleration, data_out->angular_acceleration);
    tf::vectorEigenToMsg(angle_error, data_out->angle_error);
    tf::vectorEigenToMsg(angular_rate_error, data_out->angle_rate_error);
  }
}




}
