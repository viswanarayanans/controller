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

#include "rrc_control/elas_smc_position_controller.h"
#include <eigen_conversions/eigen_msg.h>


namespace rrc_control {

ElasSmcPositionController::ElasSmcPositionController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

ElasSmcPositionController::~ElasSmcPositionController() {}

void ElasSmcPositionController::InitializeParameters() {
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));

  m_hat = 4*vehicle_parameters_.scissors_mass_ + 4*vehicle_parameters_.rotor_unit_mass_ 
          + 4*vehicle_parameters_.rotor_mass_ + 4*vehicle_parameters_.battery_mass_;
  Fp = vehicle_parameters_.box_mass_;

  Eigen::Vector3d half_box_hat = vehicle_parameters_.box_hat_/2;
  Eigen::Vector3d half_arm_hat;
  half_arm_hat << half_box_hat[0] + vehicle_parameters_.rotor_ext_len_/1.4142136,
                    half_box_hat[1] + vehicle_parameters_.rotor_ext_len_/1.4142136,
                    0;
  Eigen::Vector3d half_batt_hat;
  half_batt_hat << half_box_hat[0] + vehicle_parameters_.battery_dim_[0]/(2*1.4142136),
                    half_box_hat[1] + vehicle_parameters_.battery_dim_[0]/(2*1.4142136),
                    0;

  double arm_length_hat = half_arm_hat[0]*1.4142136;
  Eigen::Vector3d half_box_hat_2 = half_box_hat.cwiseProduct(half_box_hat);
  Eigen::Vector3d half_arm_hat_2 = half_arm_hat.cwiseProduct(half_arm_hat);
  Eigen::Vector3d half_batt_hat_2 = half_batt_hat.cwiseProduct(half_batt_hat);

  Eigen::Vector3d I_p_hat;
  I_p_hat << 4*vehicle_parameters_.scissors_mass_*(half_box_hat_2[1] + half_box_hat_2[2])/12,
             4*vehicle_parameters_.scissors_mass_*(half_box_hat_2[0] + half_box_hat_2[2])/12,
             4*vehicle_parameters_.scissors_mass_*(half_box_hat_2[0] + half_box_hat_2[1])/12;

  Eigen::Vector3d I_u_hat;
  I_u_hat << 4*vehicle_parameters_.rotor_unit_mass_*(half_box_hat_2[1]),
             4*vehicle_parameters_.rotor_unit_mass_*(half_box_hat_2[0]),
             4*vehicle_parameters_.rotor_unit_mass_*(half_box_hat_2[0] + half_box_hat_2[1]);

  Eigen::Vector3d I_r_hat;
  I_r_hat << 4*vehicle_parameters_.rotor_mass_*(half_arm_hat_2[1]),
             4*vehicle_parameters_.rotor_mass_*(half_arm_hat_2[0]),
             4*vehicle_parameters_.rotor_mass_*(half_arm_hat_2[0] + half_arm_hat_2[1]);

  Eigen::Vector3d I_b_hat;
  I_b_hat << 4*vehicle_parameters_.battery_mass_*(half_batt_hat_2[1]),
             4*vehicle_parameters_.battery_mass_*(half_batt_hat_2[0]),
             4*vehicle_parameters_.battery_mass_*(half_batt_hat_2[0] + half_batt_hat_2[1]);

  Eigen::Vector3d I_hat = I_p_hat + I_u_hat + I_r_hat + I_b_hat;
  J_hat[0] = I_hat[0]/arm_length_hat;
  J_hat[1] = I_hat[1]/arm_length_hat;
  J_hat[2] = I_hat[2];

  C_hat[0] = (I_hat[2]-I_hat[1])/arm_length_hat;
  C_hat[1] = (I_hat[0]-I_hat[2])/arm_length_hat;
  C_hat[2] = (I_hat[1]-I_hat[0]);

  

  Eigen::Vector3d half_box = vehicle_parameters_.box_dim_/2;
  Eigen::Vector3d half_arm;
  half_arm << half_box[0] + vehicle_parameters_.rotor_ext_len_/1.4142136,
                    half_box[1] + vehicle_parameters_.rotor_ext_len_/1.4142136,
                    0;
  Eigen::Vector3d half_batt;
  half_batt << half_box[0] + vehicle_parameters_.battery_dim_[0]/(2*1.4142136),
                    half_box[1] + vehicle_parameters_.battery_dim_[0]/(2*1.4142136),
                    0;

  double arm_length = half_arm[0]*1.4142136;
  Eigen::Vector3d half_box_2 = half_box.cwiseProduct(half_box);
  Eigen::Vector3d half_arm_2 = half_arm.cwiseProduct(half_arm);
  Eigen::Vector3d half_batt_2 = half_batt.cwiseProduct(half_batt);

  Eigen::Vector3d I_p;
  I_p << 4*vehicle_parameters_.scissors_mass_*(half_box_2[1] + half_box_2[2])/12,
             4*vehicle_parameters_.scissors_mass_*(half_box_2[0] + half_box_2[2])/12,
             4*vehicle_parameters_.scissors_mass_*(half_box_2[0] + half_box_2[1])/12;

  Eigen::Vector3d I_u;
  I_u << 4*vehicle_parameters_.rotor_unit_mass_*(half_box_2[1]),
             4*vehicle_parameters_.rotor_unit_mass_*(half_box_2[0]),
             4*vehicle_parameters_.rotor_unit_mass_*(half_box_2[0] + half_box_2[1]);

  Eigen::Vector3d I_r;
  I_r << 4*vehicle_parameters_.rotor_mass_*(half_arm_2[1]),
             4*vehicle_parameters_.rotor_mass_*(half_arm_2[0]),
             4*vehicle_parameters_.rotor_mass_*(half_arm_2[0] + half_arm_2[1]);

  Eigen::Vector3d I_b;
  I_b << 4*vehicle_parameters_.battery_mass_*(half_batt_2[1]),
             4*vehicle_parameters_.battery_mass_*(half_batt_2[0]),
             4*vehicle_parameters_.battery_mass_*(half_batt_2[0] + half_batt_2[1]);

  Eigen::Vector3d I = I_p + I_u + I_r + I_b;
  Eigen::Vector3d J;
  J[0] = I[0]/arm_length;
  J[1] = I[1]/arm_length;
  J[2] = I[2];

  Eigen::Vector3d C;
  C[0] = (I[2]-I[1])/arm_length;
  C[1] = (I[0]-I[2])/arm_length;
  C[2] = (I[1]-I[0]);

  Fq1 = J - J_hat;
  Fq2 = C - C_hat;
  Fq1 << 0, 0, 0;
  Fq2 << 0, 0, 0;
  
  rotor_vel_coef_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  rotor_vel_coef_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse();
  initialized_params_ = true;
}

void ElasSmcPositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities, 
                                                     rrc_control::PlotDataMsg* data_out) const {
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
  ROS_INFO_STREAM(thrust);

  Eigen::Vector3d moments;
  CalculateMoments(thrust_3d, &moments, data_out);
  // ROS_INFO_STREAM("moments: "<<moments[0]<<moments[1]<<moments[2]);
  tf::vectorEigenToMsg(moments, data_out->moments);


  Eigen::Vector4d moment_thrust;
  moment_thrust.block<3, 1>(0, 0) = moments;
  moment_thrust(3) = thrust;

  *rotor_velocities = rotor_vel_coef_ * moment_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}

void ElasSmcPositionController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void ElasSmcPositionController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void ElasSmcPositionController::ComputeThrust(Eigen::Vector3d* thrust,
                                          rrc_control::PlotDataMsg* data_out) const {
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

  Eigen::Vector3d up_hat = m_hat*(vehicle_parameters_.gravity_ * e_3 + 
                          command_trajectory_.acceleration_W - 
                          controller_parameters_.theta_p_.cwiseProduct(velocity_error));

  Eigen::Vector3d kp = controller_parameters_.eeta_p_ + 
                        Fp*(vehicle_parameters_.gravity_ * e_3 + 
                          command_trajectory_.acceleration_W - 
                          controller_parameters_.theta_p_.cwiseProduct(velocity_error));

  Eigen::Vector3d delTau_p;
  delTau_p << kp[0]*Sigmoid(sp[0], controller_parameters_.var_pi_p_),
              kp[1]*Sigmoid(sp[1], controller_parameters_.var_pi_p_),
              kp[2]*Sigmoid(sp[2], controller_parameters_.var_pi_p_);


  *thrust = up_hat - delTau_p;

  tf::vectorEigenToMsg(position_error, data_out->position_error);
  tf::vectorEigenToMsg(velocity_error, data_out->velocity_error);

  tf::vectorEigenToMsg(kp, data_out->Kp1_hat);
  tf::vectorEigenToMsg(sp, data_out->sp);
  tf::vectorEigenToMsg(delTau_p, data_out->delTau_p);
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void ElasSmcPositionController::CalculateMoments(Eigen::Vector3d force, 
                        Eigen::Vector3d* moments,
                        rrc_control::PlotDataMsg* data_out) const {
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

    Eigen::Matrix3d C_hat_;
    C_hat_ << 0, 0, C_hat[0]*odometry_.angular_velocity[1],
              C_hat[1]*odometry_.angular_velocity[2], 0, 0,
              0, C_hat[2]*odometry_.angular_velocity[0], 0;

    Eigen::Vector3d uq_hat = C_hat_ * odometry_.angular_velocity + 
                              J_hat.cwiseProduct(R_des.transpose() * R * 
                                command_trajectory_.angular_acceleration_W - 
                                controller_parameters_.theta_q_.cwiseProduct(angular_rate_error));

    Eigen::Matrix3d Fq2_;                     
    Fq2_ << 0, 0, Fq2[0]*odometry_.angular_velocity[1],
              Fq2[1]*odometry_.angular_velocity[2], 0, 0,
              0, Fq2[2]*odometry_.angular_velocity[0], 0;

    Eigen::Vector3d kq = controller_parameters_.eeta_q_ +
                          Fq2_ * odometry_.angular_velocity +
                          Fq1.cwiseProduct(R_des.transpose() * R * 
                          command_trajectory_.angular_acceleration_W - 
                          controller_parameters_.theta_q_.cwiseProduct(angular_rate_error));

    Eigen::Vector3d delTau_q;
    delTau_q << kq[0]*Sigmoid(sq[0], controller_parameters_.var_pi_q_),
                kq[1]*Sigmoid(sq[1], controller_parameters_.var_pi_q_),
                kq[2]*Sigmoid(sq[2], controller_parameters_.var_pi_q_);

    *moments = uq_hat - delTau_q;

    tf::vectorEigenToMsg(angle_error, data_out->angle_error);
    tf::vectorEigenToMsg(angular_rate_error, data_out->angle_rate_error);

    tf::vectorEigenToMsg(kq, data_out->Kq1_hat_0);
    tf::vectorEigenToMsg(sq, data_out->sq);
    tf::vectorEigenToMsg(delTau_q, data_out->delTau_q);
  }
}


double ElasSmcPositionController::Sigmoid(double s, double var_pi) const {
  return ((abs(s) > var_pi) ? s/abs(s) : s/var_pi);
}


}