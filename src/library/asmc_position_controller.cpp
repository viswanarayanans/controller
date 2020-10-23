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

#include "rrc_control/asmc_position_controller.h"
#include <eigen_conversions/eigen_msg.h>


namespace rrc_control {

ASmcPositionController::ASmcPositionController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

ASmcPositionController::~ASmcPositionController() {}

void ASmcPositionController::InitializeParameters() {
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  rotor_vel_coef_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  rotor_vel_coef_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse();
  initialized_params_ = true;
}

void ASmcPositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities, 
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
  ROS_INFO_STREAM(thrust);
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

void ASmcPositionController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void ASmcPositionController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void ASmcPositionController::CalculateThrust(Eigen::Vector3d* thrust, 
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
  ros::Time current_time;

  Eigen::Vector3d sp;
  sp = velocity_error + controller_parameters_.theta_p_.cwiseProduct(position_error);

  static Eigen::Vector3d hatKp = controller_parameters_.hatKp_;
  static double hatM = controller_parameters_.hatM_;

  Eigen::Vector3d dot_hatKp;
  double dot_hatM;

  dot_hatKp = sp.cwiseAbs() - controller_parameters_.alpha_p_.cwiseProduct(hatKp);
  // dot_hatM = vehicle_parameters_.gravity_*abs(sp[2]) - controller_parameters_.alpha_m_*(hatM);
  dot_hatM = -vehicle_parameters_.gravity_*sp[2] - controller_parameters_.alpha_m_*(hatM);


  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  
  hatKp += dot_hatKp*dt;
  hatM += dot_hatM*dt;

  last_time = current_time;

  Eigen::Matrix3d lam_p = controller_parameters_.lam_p_.asDiagonal();
  Eigen::Vector3d rho_p = hatKp;

  Eigen::Vector3d delTau_p;
  delTau_p << rho_p[0]*Sigmoid(sp[0], controller_parameters_.var_pi_p_), 
				  rho_p[1]*Sigmoid(sp[1], controller_parameters_.var_pi_p_), 
				  rho_p[2]*Sigmoid(sp[2], controller_parameters_.var_pi_p_);
  
  *thrust =  -lam_p*sp - delTau_p + hatM*vehicle_parameters_.gravity_*e_3;
  
  tf::vectorEigenToMsg(position_error, data_out->position_error);
  tf::vectorEigenToMsg(velocity_error, data_out->velocity_error);

  tf::vectorEigenToMsg(hatKp, data_out->Kp_hat);
  tf::vectorEigenToMsg(sp, data_out->sp);
  tf::vectorEigenToMsg(rho_p, data_out->rho_p);
  tf::vectorEigenToMsg(delTau_p, data_out->delTau_p);

  data_out->M_hat = hatM;
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void ASmcPositionController::CalculateMoments(Eigen::Vector3d force, 
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
	  
	  // ROS_INFO_STREAM(xqx, xqy);

	  Eigen::Vector3d dot_hatKq_0, dot_hatKq_1, dot_hatKq_2;
	  Eigen::Vector3d xq_norm;
	  xq_norm = angle_error.cwiseAbs();

	  // ROS_INFO_STREAM("XQ_NORM" << xq_norm);

	  Eigen::Vector3d sq_norm = sq.cwiseAbs();
	  // Eigen::Vector3d sq_norm = angle_error.cwiseAbs();


	  // ROS_INFO_STREAM("SQ_NORM" << sq_norm);
	  // ROS_INFO("SQ and XQ done");

	  static Eigen::Vector3d hatKq_0 = controller_parameters_.hatKq_0_;
	  static Eigen::Vector3d hatKq_1 = controller_parameters_.hatKq_1_;
	  static Eigen::Vector3d hatKq_2 = controller_parameters_.hatKq_2_;

	  Eigen::Vector3d xqx_norm, xqy_norm, xqz_norm;
	  xqx_norm << 1, xq_norm[0], pow(xq_norm[0],2);
	  xqy_norm << 1, xq_norm[1], pow(xq_norm[1],2);
	  xqz_norm << 1, xq_norm[2], pow(xq_norm[2],2);

	  // ROS_INFO_STREAM("XQXYZ/ done" << xqx_norm);

	  dot_hatKq_0 = sq_norm[0]*xqx_norm - controller_parameters_.alpha_q0_.cwiseProduct(hatKq_0);
	  dot_hatKq_1 = sq_norm[1]*xqy_norm - controller_parameters_.alpha_q1_.cwiseProduct(hatKq_1);
	  dot_hatKq_2 = sq_norm[2]*xqz_norm - controller_parameters_.alpha_q2_.cwiseProduct(hatKq_2);


	  current_time = ros::Time::now();
	  double dt = (current_time - last_time).toSec();
	  
	  hatKq_0 += dot_hatKq_0*dt;
	  hatKq_1 += dot_hatKq_1*dt;
	  hatKq_2 += dot_hatKq_2*dt;
	  
	  last_time = current_time;

	  Eigen::Vector3d rho_q;
	  rho_q[0] = hatKq_0.dot(xqx_norm);
	  rho_q[1] = hatKq_1.dot(xqy_norm);
	  rho_q[2] = hatKq_2.dot(xqz_norm);

	  // ROS_INFO("RHo done");

	  Eigen::Vector3d delTau_q;

	  // delTau_q << rho_q[0]*Sigmoid(sq[0]), rho_q[1]*Sigmoid(sq[1]), rho_q[2]*Sigmoid(sq[2]);
	  delTau_q << rho_q[0]*Sigmoid(sq[0], controller_parameters_.var_pi_q_), 
				  	rho_q[1]*Sigmoid(sq[1], controller_parameters_.var_pi_q_), 
				  	rho_q[2]*Sigmoid(sq[2], controller_parameters_.var_pi_q_);

	  // ROS_INFO_STREAM(delTau_q);

	  *moments = -controller_parameters_.lam_q_.cwiseProduct(sq) - delTau_q;

    tf::vectorEigenToMsg(angle_error, data_out->angle_error);
    tf::vectorEigenToMsg(angular_rate_error, data_out->angle_rate_error);

    tf::vectorEigenToMsg(hatKq_0, data_out->Kq_hat_0);
    tf::vectorEigenToMsg(hatKq_1, data_out->Kq_hat_1);
    tf::vectorEigenToMsg(hatKq_2, data_out->Kq_hat_2);
    tf::vectorEigenToMsg(sq, data_out->sq);
    tf::vectorEigenToMsg(rho_q, data_out->rho_q);
    tf::vectorEigenToMsg(delTau_q, data_out->delTau_q);

  }
}


double ASmcPositionController::Sigmoid(double s, double var_pi) const {
	return ((abs(s) > var_pi) ? s/abs(s) : s/var_pi);
}


}
