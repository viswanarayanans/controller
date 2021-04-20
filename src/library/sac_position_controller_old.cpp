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

#include "rrc_control/sac_position_controller.h"
#include <eigen_conversions/eigen_msg.h>


namespace rrc_control {

SacPositionController::SacPositionController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

SacPositionController::~SacPositionController() {}

void SacPositionController::InitializeParameters() {
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_attitude_gain_ = (controller_parameters_.lam_q_1_.cwiseProduct(controller_parameters_.theta_q_)).transpose()
      * vehicle_parameters_.inertia_.inverse();
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_angular_rate_gain_ = controller_parameters_.lam_q_1_.transpose()
      * vehicle_parameters_.inertia_.inverse();


  rotor_vel_coef_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  rotor_vel_coef_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse();

  double theta_p_min = controller_parameters_.theta_p_.minCoeff();
  double theta_q_min = controller_parameters_.theta_q_.minCoeff();
  
  varrho_p_1 = (Minimum(controller_parameters_.lam_p_1_.minCoeff(), theta_p_min))/2;
  varrho_p_2 = (Minimum(controller_parameters_.lam_p_2_.minCoeff(), theta_p_min))/2.5;
  varrho_q_1 = Minimum(controller_parameters_.lam_q_1_.minCoeff(), theta_q_min);
  varrho_q_2 = Minimum(controller_parameters_.lam_q_2_.minCoeff(), theta_q_min);

  state_ = 1;

  initialized_params_ = true;
}

void SacPositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities, 
                        msg_check::PlotDataMsg* data_out) {
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

void SacPositionController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void SacPositionController::SetState(const int& state) {
  state_ = state;
  ROS_INFO_STREAM("Switching to state: "<< state_);
}

void SacPositionController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void SacPositionController::CalculateThrust(Eigen::Vector3d* thrust, 
                        msg_check::PlotDataMsg* data_out) {
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

  Eigen::Vector3d sp, sp_abs;
  sp = velocity_error + controller_parameters_.theta_p_.cwiseProduct(position_error);
  sp_abs = sp.cwiseAbs();

  Eigen::Vector3d xi_p_sq, xi_p;
  xi_p_sq = position_error.cwiseProduct(position_error);
  xi_p = xi_p_sq.cwiseSqrt();

  Eigen::Vector3d delTau_p, zeta_p;
  // static Eigen::Vector3d hatKp0_1 = controller_parameters_.hatKp0_1_;
  // static Eigen::Vector3d hatKp1_1 = controller_parameters_.hatKp1_1_;
  // static Eigen::Vector3d hatKp0_2 = controller_parameters_.hatKp0_2_;
  // static Eigen::Vector3d hatKp1_2 = controller_parameters_.hatKp1_2_;
  // static Eigen::Vector3d rho_p0_1 = controller_parameters_.rho_p0_1_;
  // static Eigen::Vector3d rho_p1_1 = controller_parameters_.rho_p1_1_;
  // static Eigen::Vector3d rho_p0_2 = controller_parameters_.rho_p0_2_;
  // static Eigen::Vector3d rho_p1_2 = controller_parameters_.rho_p1_2_;
  // static double hatM_1 = controller_parameters_.hatM_1_;
  // static double hatM_2 = controller_parameters_.hatM_2_;

  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  if (dt < 0){
    dt = 0;
  }
  else if (dt > 0.1){
    dt = 0.1;
  }

  // *thrust = -(position_error.cwiseProduct(controller_parameters_.theta_p_.cwiseProduct(controller_parameters_.lam_p_1_))
  //       + velocity_error.cwiseProduct(controller_parameters_.lam_p_1_)
  //       - (vehicle_parameters_.gravity_ * e_3 - command_trajectory_.acceleration_W)*vehicle_parameters_.mass_);
    

  if(state_==1){
    Eigen::Vector3d dot_hatKp0_1 = sp_abs - controller_parameters_.alpha_p0_.cwiseProduct(controller_parameters_.hatKp0_1_);
    Eigen::Vector3d dot_hatKp1_1 = sp_abs.cwiseProduct(xi_p) - controller_parameters_.alpha_p1_.cwiseProduct(controller_parameters_.hatKp1_1_);
    Eigen::Vector3d dot_rho_p0_2 = -(controller_parameters_.beta_p0_2_ 
                                    + (varrho_p_2/2)*controller_parameters_.hatKp0_2_.cwiseProduct(controller_parameters_.rho_p0_2_))
                                    + controller_parameters_.beta_p0_2_.cwiseProduct(controller_parameters_.nu_p0_2_);
    Eigen::Vector3d dot_rho_p1_2 = -(controller_parameters_.beta_p1_2_ 
                                    + (varrho_p_2/2)*controller_parameters_.hatKp1_2_.cwiseProduct(controller_parameters_.rho_p1_2_))
                                    + controller_parameters_.beta_p1_2_.cwiseProduct(controller_parameters_.nu_p1_2_);
    double dot_hatM_1 = -vehicle_parameters_.gravity_*sp[2] - controller_parameters_.alpha_m_*controller_parameters_.hatM_1_;


    controller_parameters_.hatKp0_1_ += dot_hatKp0_1*dt;
    controller_parameters_.hatKp1_1_<< 0, 0, 0;

    controller_parameters_.rho_p0_2_ += dot_rho_p0_2*dt;
    controller_parameters_.rho_p1_2_ += dot_rho_p1_2*dt;

    controller_parameters_.hatM_1_ += dot_hatM_1*dt;
    ROS_INFO_STREAM(controller_parameters_.hatM_1_);
    
    // if(controller_parameters_.hatM_1_<0){
    //   controller_parameters_.hatM_1_ = 0;
    // }

    zeta_p = (controller_parameters_.hatKp0_1_ + controller_parameters_.rho_p0_1_) + (controller_parameters_.hatKp1_1_ + controller_parameters_.rho_p1_1_).cwiseProduct(xi_p);

    delTau_p << zeta_p[0]*Sigmoid(sp[0], controller_parameters_.var_pi_p_), 
          zeta_p[1]*Sigmoid(sp[1], controller_parameters_.var_pi_p_), 
          zeta_p[2]*Sigmoid(sp[2], controller_parameters_.var_pi_p_);
    *thrust = -controller_parameters_.lam_p_1_.cwiseProduct(sp) - delTau_p + controller_parameters_.hatM_1_*vehicle_parameters_.gravity_*e_3;
  }

  else if(state_==2){
    Eigen::Vector3d dot_hatKp0_2 = sp_abs - controller_parameters_.alpha_p0_.cwiseProduct(controller_parameters_.hatKp0_2_);
    Eigen::Vector3d dot_hatKp1_2 = sp_abs.cwiseProduct(xi_p) - controller_parameters_.alpha_p1_.cwiseProduct(controller_parameters_.hatKp1_2_);
    Eigen::Vector3d dot_rho_p0_1 = -(controller_parameters_.beta_p0_1_ 
                                    + (varrho_p_1/2)*controller_parameters_.hatKp0_1_.cwiseProduct(controller_parameters_.rho_p0_1_))
                                    + controller_parameters_.beta_p0_1_.cwiseProduct(controller_parameters_.nu_p0_1_);
    Eigen::Vector3d dot_rho_p1_1 = -(controller_parameters_.beta_p1_1_ 
                                    + (varrho_p_1/2)*controller_parameters_.hatKp1_1_.cwiseProduct(controller_parameters_.rho_p1_1_))
                                    + controller_parameters_.beta_p1_1_.cwiseProduct(controller_parameters_.nu_p1_1_);
    double dot_hatM_2 = -vehicle_parameters_.gravity_*sp[2] - controller_parameters_.alpha_m_*controller_parameters_.hatM_2_;


    controller_parameters_.hatKp0_2_ += dot_hatKp0_2*dt;
    controller_parameters_.hatKp1_2_ += dot_hatKp1_2*dt;

    controller_parameters_.rho_p0_1_ += dot_rho_p0_1*dt;
    controller_parameters_.rho_p1_1_ += dot_rho_p1_1*dt;

    controller_parameters_.hatM_2_ += dot_hatM_2*dt;

    zeta_p = (controller_parameters_.hatKp0_2_ + controller_parameters_.rho_p0_2_) + (controller_parameters_.hatKp1_2_ + controller_parameters_.rho_p1_2_).cwiseProduct(xi_p);

    delTau_p << zeta_p[0]*Sigmoid(sp[0], controller_parameters_.var_pi_p_), 
          zeta_p[1]*Sigmoid(sp[1], controller_parameters_.var_pi_p_), 
          zeta_p[2]*Sigmoid(sp[2], controller_parameters_.var_pi_p_);
    *thrust = -controller_parameters_.lam_p_2_.cwiseProduct(sp) - position_error - delTau_p + controller_parameters_.hatM_2_*vehicle_parameters_.gravity_*e_3;
  }

  else{
    *thrust << 0, 0, 0;
  }
  // ROS_INFO_STREAM("Forces:"<< std::endl << *thrust);

  // if (thrust->z() > 20*vehicle_parameters_.mass_){
  //   thrust->z() = 20*vehicle_parameters_.mass_;
  // }
  // else if (thrust->z() < -20*vehicle_parameters_.mass_){
  //   thrust->z() = -20*vehicle_parameters_.mass_;
  // }

  // if (thrust->x() > 4*vehicle_parameters_.mass_){
  //   thrust->x() = 4*vehicle_parameters_.mass_;
  // }
  // else if (thrust->x() < -4*vehicle_parameters_.mass_){
  //   thrust->x() = -4*vehicle_parameters_.mass_;
  // }

  // if (thrust->y() > 4*vehicle_parameters_.mass_){
  //   thrust->y() = 4*vehicle_parameters_.mass_;
  // }
  // else if (thrust->y() < -4*vehicle_parameters_.mass_){
  //   thrust->y() = -4*vehicle_parameters_.mass_;
  // }

  // // static Eigen::Vector3d hatKp = controller_parameters_.hatKp_;
  // // static double hatM = controller_parameters_.hatM_;

  // // Eigen::Vector3d dot_hatKp;
  // // double dot_hatM;

  // // dot_hatKp = sp.cwiseAbs() - controller_parameters_.alpha_p_.cwiseProduct(hatKp);
  // // // dot_hatM = vehicle_parameters_.gravity_*abs(sp[2]) - controller_parameters_.alpha_m_*(hatM);
  // // dot_hatM = -vehicle_parameters_.gravity_*sp[2] - controller_parameters_.alpha_m_*(hatM);

  
  // // hatKp += dot_hatKp*dt;
  // // hatM += dot_hatM*dt;

  // // last_time = current_time;

  // // Eigen::Matrix3d lam_p = controller_parameters_.lam_p_.asDiagonal();
  // // Eigen::Vector3d rho_p = hatKp;

  // // Eigen::Vector3d delTau_p;
  // // delTau_p << rho_p[0]*Sigmoid(sp[0], controller_parameters_.var_pi_p_), 
		// // 		  rho_p[1]*Sigmoid(sp[1], controller_parameters_.var_pi_p_), 
		// // 		  rho_p[2]*Sigmoid(sp[2], controller_parameters_.var_pi_p_);
  
  // // *thrust =  -lam_p*sp - delTau_p + hatM*vehicle_parameters_.gravity_*e_3;
  
  // // tf::vectorEigenToMsg(position_error, data_out->position_error);
  // // tf::vectorEigenToMsg(velocity_error, data_out->velocity_error);

  // // tf::vectorEigenToMsg(hatKp, data_out->Kp_hat);
  // // tf::vectorEigenToMsg(sp, data_out->sp);
  // // tf::vectorEigenToMsg(rho_p, data_out->rho_p);
  // // tf::vectorEigenToMsg(delTau_p, data_out->delTau_p);

  // // data_out->M_hat = hatM;
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void SacPositionController::CalculateMoments(Eigen::Vector3d force, 
												Eigen::Vector3d* moments, 
                        msg_check::PlotDataMsg* data_out) {
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

	  Eigen::Vector3d sq_abs = sq.cwiseAbs();
	  // Eigen::Vector3d sq_norm = angle_error.cwiseAbs();


	  // ROS_INFO_STREAM("SQ_NORM" << sq_norm);
	  // ROS_INFO("SQ and XQ done");
    Eigen::Vector3d xi_q_sq = angle_error.cwiseProduct(angle_error) 
                          + angular_rate_error.cwiseProduct(angular_rate_error);
    Eigen::Vector3d xi_q = xi_q_sq.cwiseSqrt();

	  // ROS_INFO_STREAM("XQXYZ/ done" << xqx_norm);
    
    

    Eigen::Vector3d delTau_q, zeta_q;
    // static Eigen::Vector3d hatKq0_1 = controller_parameters_.hatKq0_1_;
    // static Eigen::Vector3d hatKq1_1 = controller_parameters_.hatKq1_1_;
    // static Eigen::Vector3d hatKq2_1 = controller_parameters_.hatKq2_1_;
    // static Eigen::Vector3d hatKq0_2 = controller_parameters_.hatKq0_2_;
    // static Eigen::Vector3d hatKq1_2 = controller_parameters_.hatKq1_2_;
    // static Eigen::Vector3d hatKq2_2 = controller_parameters_.hatKq2_2_;
    // static Eigen::Vector3d rho_q0_1 = controller_parameters_.rho_q0_1_;
    // static Eigen::Vector3d rho_q1_1 = controller_parameters_.rho_q1_1_;
    // static Eigen::Vector3d rho_q2_1 = controller_parameters_.rho_q2_1_;
    // static Eigen::Vector3d rho_q0_2 = controller_parameters_.rho_q0_2_;
    // static Eigen::Vector3d rho_q1_2 = controller_parameters_.rho_q1_2_;
    // static Eigen::Vector3d rho_q2_2 = controller_parameters_.rho_q2_2_;

	  current_time = ros::Time::now();
	  double dt = (current_time - last_time).toSec();

    // if (dt<0){
    //   dt = 0;
    // }
    // else if (dt > 0.1){
    //   dt = 0.1;
    // }

    *moments = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
                           - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
                           + odometry_.angular_velocity.cross(odometry_.angular_velocity);

    *moments = vehicle_parameters_.inertia_ * *moments;

    // if (state_==1){
    //   Eigen::Vector3d dot_hatKq0_1 = sq_abs - controller_parameters_.alpha_q0_.cwiseProduct(hatKq0_1);
    //   Eigen::Vector3d dot_hatKq1_1 = sq_abs.cwiseProduct(xi_q) - controller_parameters_.alpha_q1_.cwiseProduct(hatKq1_1);
    //   Eigen::Vector3d dot_hatKq2_1 = sq_abs.cwiseProduct(xi_q_sq) - controller_parameters_.alpha_q2_.cwiseProduct(hatKq2_1);

    //   Eigen::Vector3d dot_rho_q0_2 = -(controller_parameters_.beta_q0_2_ + (varrho_q_2/2)*hatKq0_2.cwiseProduct(rho_q0_2))
    //                                   + controller_parameters_.beta_q0_2_.cwiseProduct(controller_parameters_.nu_q0_2_);
    //   Eigen::Vector3d dot_rho_q1_2 = -(controller_parameters_.beta_q1_2_ + (varrho_p_2/2)*hatKq1_2.cwiseProduct(rho_q1_2))
    //                                   + controller_parameters_.beta_q1_2_.cwiseProduct(controller_parameters_.nu_q1_2_);
    //   Eigen::Vector3d dot_rho_q2_2 = -(controller_parameters_.beta_q2_2_ + (varrho_p_2/2)*hatKq2_2.cwiseProduct(rho_q2_2))
    //                                   + controller_parameters_.beta_q2_2_.cwiseProduct(controller_parameters_.nu_q2_2_);

    //   hatKq0_1 += dot_hatKq0_1*dt;
    //   hatKq1_1 += dot_hatKq1_1*dt;
    //   hatKq2_1 += dot_hatKq2_1*dt;

    //   rho_q0_2 += dot_rho_q0_2*dt;
    //   rho_q1_2 += dot_rho_q1_2*dt;
    //   rho_q2_2 += dot_rho_q2_2*dt;

    //   zeta_q = (hatKq0_1 + rho_q0_1) + (hatKq1_1 + rho_q1_1).cwiseProduct(xi_q)
    //           + (hatKq2_1 + rho_q2_1).cwiseProduct(xi_q_sq);

    //   delTau_q << zeta_q[0]*Sigmoid(sq[0], controller_parameters_.var_pi_q_), 
    //         zeta_q[1]*Sigmoid(sq[1], controller_parameters_.var_pi_q_), 
    //         zeta_q[2]*Sigmoid(sq[2], controller_parameters_.var_pi_q_);
    //   *moments = -controller_parameters_.lam_q_1_.cwiseProduct(sq) - delTau_q;
    // }

    // else if (state_==2){
    //   Eigen::Vector3d dot_hatKq0_2 = sq_abs - controller_parameters_.alpha_q0_.cwiseProduct(hatKq0_2);
    //   Eigen::Vector3d dot_hatKq1_2 = sq_abs.cwiseProduct(xi_q) - controller_parameters_.alpha_q1_.cwiseProduct(hatKq1_2);
    //   Eigen::Vector3d dot_hatKq2_2 = sq_abs.cwiseProduct(xi_q_sq) - controller_parameters_.alpha_q2_.cwiseProduct(hatKq2_2);

    //   Eigen::Vector3d dot_rho_q0_1 = -(controller_parameters_.beta_q0_1_ + (varrho_q_1/2)*hatKq0_1.cwiseProduct(rho_q0_1))
    //                                   + controller_parameters_.beta_q0_1_.cwiseProduct(controller_parameters_.nu_q0_1_);
    //   Eigen::Vector3d dot_rho_q1_1 = -(controller_parameters_.beta_q1_1_ + (varrho_p_1/2)*hatKq1_1.cwiseProduct(rho_q1_1))
    //                                   + controller_parameters_.beta_q1_1_.cwiseProduct(controller_parameters_.nu_q1_1_);
    //   Eigen::Vector3d dot_rho_q2_1 = -(controller_parameters_.beta_q2_1_ + (varrho_p_1/2)*hatKq2_1.cwiseProduct(rho_q2_1))
    //                                   + controller_parameters_.beta_q2_1_.cwiseProduct(controller_parameters_.nu_q2_1_);

    //   hatKq0_2 += dot_hatKq0_2*dt;
    //   hatKq1_2 += dot_hatKq1_2*dt;
    //   hatKq2_2 += dot_hatKq2_2*dt;

    //   rho_q0_1 += dot_rho_q0_1*dt;
    //   rho_q1_1 += dot_rho_q1_1*dt;
    //   rho_q2_1 += dot_rho_q2_1*dt;

    //   zeta_q = (hatKq0_2 + rho_q0_2) + (hatKq1_2 + rho_q1_2).cwiseProduct(xi_q)
    //           + (hatKq2_2 + rho_q2_2).cwiseProduct(xi_q_sq);

    //   delTau_q << zeta_q[0]*Sigmoid(sq[0], controller_parameters_.var_pi_q_), 
    //         zeta_q[1]*Sigmoid(sq[1], controller_parameters_.var_pi_q_), 
    //         zeta_q[2]*Sigmoid(sq[2], controller_parameters_.var_pi_q_);
    //   *moments = -controller_parameters_.lam_q_2_.cwiseProduct(sq) - angle_error - delTau_q;
    // }
    // else {
    //   *moments << 0, 0, 0;
    // }
   //  tf::vectorEigenToMsg(angle_error, data_out->angle_error);
   //  tf::vectorEigenToMsg(angular_rate_error, data_out->angle_rate_error);

   //  tf::vectorEigenToMsg(hatKq_0, data_out->Kq_hat0);
   //  tf::vectorEigenToMsg(hatKq_1, data_out->Kq_hat1);
   //  tf::vectorEigenToMsg(hatKq_2, data_out->Kq_hat2);
   //  tf::vectorEigenToMsg(sq, data_out->sq);
   //  tf::vectorEigenToMsg(rho_q, data_out->rho_q);
   //  tf::vectorEigenToMsg(delTau_q, data_out->delTau_q);

  }
}


inline double SacPositionController::Sigmoid(double s, double var_pi) const {
	return ((abs(s) > var_pi) ? s/abs(s) : s/var_pi);
}

inline double SacPositionController::Minimum(double x, double y) const {
  return ((x > y) ? x : y);
}


}
