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

#include "rrc_control/smc_position_controller.h"
#include <eigen_conversions/eigen_msg.h>


namespace rrc_control {

SmcPositionController::SmcPositionController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

SmcPositionController::~SmcPositionController() {}

void SmcPositionController::InitializeParameters() {
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  controller_parameters_.Fp_ = controller_parameters_.payload_;

  double ms2 = controller_parameters_.payload_ * controller_parameters_.side_ * controller_parameters_.side_ / 6;
  double md2 = controller_parameters_.payload_ * controller_parameters_.length_ * controller_parameters_.length_;
  double ml2 = controller_parameters_.rotor_mass_ * controller_parameters_.arm_length_ * controller_parameters_.arm_length_;
  
  controller_parameters_.inertia_(0,0) = 2*ml2;
  controller_parameters_.inertia_(1,1) = 2*ml2;
  controller_parameters_.inertia_(2,2) = 4*ml2;

  controller_parameters_.J_cap_ << controller_parameters_.inertia_(0,0) / controller_parameters_.length_,
                                    controller_parameters_.inertia_(1,1) / controller_parameters_.length_,
                                    controller_parameters_.inertia_(2,2);

  controller_parameters_.C_cap_ << (controller_parameters_.inertia_(2,2) - controller_parameters_.inertia_(1,1)) / controller_parameters_.length_,
                                    (controller_parameters_.inertia_(0,0) - controller_parameters_.inertia_(2,2)) / controller_parameters_.length_,
                                    (controller_parameters_.inertia_(1,1) - controller_parameters_.inertia_(0,0));


    
  controller_parameters_.Fq1_ << (ms2+md2)/(2*controller_parameters_.arm_length_),
                                  (ms2+md2)/(2*controller_parameters_.arm_length_),
                                  ms2/2;

  controller_parameters_.Fq2_ << md2/(2*controller_parameters_.arm_length_), md2/(2*controller_parameters_.arm_length_), 0;


  // To make the tuning independent of the inertia matrix we divide here.
  // normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose()
      // * vehicle_parameters_.inertia_.inverse();
  // To make the tuning independent of the inertia matrix we divide here.
  // normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose()
      // * vehicle_parameters_.inertia_.inverse();

  //Added by Viswa : To add payload mass with base mass
  // normalized_mass = vehicle_parameters_.mass_ + vehicle_parameters_.payload_mass_;

  // kp = controller_parameters_.Fp_ + controller_parameters_.eeta_p_;
  

  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
  I(3, 3) = 1;
  I.setIdentity();



  //ROS_INFO("Inertia : %f %f %f", vehicle_parameters_.inertia_(0,0), vehicle_parameters_.inertia_(1,1), vehicle_parameters_.inertia_(2,2));
  
  rotor_vel_coef_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  rotor_vel_coef_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;
  initialized_params_ = true;
}

void SmcPositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities, 
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
  ComputeThrust(&thrust_3d, data_out);
  // ROS_INFO("Thrust done");

  double thrust = thrust_3d.dot(odometry_.orientation.toRotationMatrix().col(2)); //Added by Viswa
  data_out->thrust = thrust;
  // ROS_INFO_STREAM(thrust);

  Eigen::Vector3d moments;
  CalculateMoments(thrust_3d, &moments, data_out);
  tf::vectorEigenToMsg(moments, data_out->moments);


  Eigen::Vector4d moment_thrust;
  moment_thrust.block<3, 1>(0, 0) = moments;
  moment_thrust(3) = thrust;

  *rotor_velocities = rotor_vel_coef_ * moment_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}

void SmcPositionController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void SmcPositionController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void SmcPositionController::ComputeThrust(Eigen::Vector3d* thrust,
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

  Eigen::Vector3d sp;
  sp = velocity_error + controller_parameters_.theta_p_.cwiseProduct(position_error);

  Eigen::Vector3d up_cap = controller_parameters_.m_cap_*(controller_parameters_.gravity_ * e_3 + 
                          command_trajectory_.acceleration_W - 
                          controller_parameters_.theta_p_.cwiseProduct(velocity_error));

  Eigen::Vector3d kp = controller_parameters_.eeta_p_ + 
                        (controller_parameters_.Fp_)*(controller_parameters_.gravity_ * e_3 + 
                          command_trajectory_.acceleration_W - 
                          controller_parameters_.theta_p_.cwiseProduct(velocity_error));

  Eigen::Vector3d delTau_p;
  delTau_p << kp[0]*Sigmoid(sp[0], controller_parameters_.var_pi_p_),
              kp[1]*Sigmoid(sp[1], controller_parameters_.var_pi_p_),
              kp[2]*Sigmoid(sp[2], controller_parameters_.var_pi_p_);


  *thrust = up_cap - delTau_p;

  tf::vectorEigenToMsg(position_error, data_out->position_error);
  tf::vectorEigenToMsg(velocity_error, data_out->velocity_error);

  tf::vectorEigenToMsg(kp, data_out->Kp_hat);
  tf::vectorEigenToMsg(sp, data_out->sp);
  tf::vectorEigenToMsg(delTau_p, data_out->delTau_p);
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void SmcPositionController::CalculateMoments(Eigen::Vector3d force, 
                        Eigen::Vector3d* moments,
                        msg_check::PlotDataMsg* data_out) const {
  assert(moments);
  // ROS_INFO_STREAM("force" << force); 
  if (force[2] >= DBL_MAX || force[2] <= -DBL_MAX) {
    *moments << 0, 0, 0;
  }

  else {

    Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();
    static ros::Time last_time = ros::Time::now();
    ros::Time current_time;

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

    // Angle error according to lee et al.
    Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
    Eigen::Vector3d angle_error;
    vectorFromSkewMatrix(angle_error_matrix, &angle_error);

    // TODO(burrimi) include angular rate references at some point.
    Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
    angular_rate_des[2] = command_trajectory_.getYawRate();

    Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;

    Eigen::Vector3d sq;
    sq = angular_rate_error + controller_parameters_.theta_q_.cwiseProduct(angle_error);

    Eigen::Matrix3d C_cap;
    C_cap << 0, 0, controller_parameters_.C_cap_[0]*odometry_.angular_velocity[1],
              controller_parameters_.C_cap_[1]*odometry_.angular_velocity[2], 0, 0,
              0, controller_parameters_.C_cap_[2]*odometry_.angular_velocity[0], 0;

    Eigen::Vector3d uq_cap = C_cap * odometry_.angular_velocity + 
                              controller_parameters_.J_cap_.cwiseProduct(R_des.transpose() * R * 
                                command_trajectory_.angular_acceleration_W - 
                                controller_parameters_.theta_q_.cwiseProduct(angular_rate_error));

    Eigen::Matrix3d Fq2;                     
    Fq2 << 0, 0, controller_parameters_.Fq2_[0]*odometry_.angular_velocity[1],
              controller_parameters_.Fq2_[1]*odometry_.angular_velocity[2], 0, 0,
              0, controller_parameters_.Fq2_[2]*odometry_.angular_velocity[0], 0;

    Eigen::Vector3d kq = controller_parameters_.eeta_q_*Eigen::Vector3d(1,1,1) +
                          Fq2 * odometry_.angular_velocity +
                          controller_parameters_.Fq1_.cwiseProduct(R_des.transpose() * R * 
                          command_trajectory_.angular_acceleration_W - 
                          controller_parameters_.theta_q_.cwiseProduct(angular_rate_error));

    Eigen::Vector3d delTau_q;
    delTau_q << kq[0]*Sigmoid(sq[0], controller_parameters_.var_pi_q_),
                kq[1]*Sigmoid(sq[1], controller_parameters_.var_pi_q_),
                kq[2]*Sigmoid(sq[2], controller_parameters_.var_pi_q_);

    *moments = uq_cap - delTau_q;

    tf::vectorEigenToMsg(angle_error, data_out->angle_error);
    tf::vectorEigenToMsg(angular_rate_error, data_out->angle_rate_error);

    tf::vectorEigenToMsg(kq, data_out->Kq_hat_0);
    tf::vectorEigenToMsg(sq, data_out->sq);
    tf::vectorEigenToMsg(delTau_q, data_out->delTau_q);
  }
}


double SmcPositionController::Sigmoid(double s, double var_pi) const {
  return ((abs(s) > var_pi) ? s/abs(s) : s/var_pi);
}


}