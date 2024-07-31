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
  rotor_vel_coef_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_attitude_gain_ = (controller_parameters_.lam_q_1_.cwiseProduct(controller_parameters_.theta_q_)).transpose()
      * vehicle_parameters_.inertia_.inverse();
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_angular_rate_gain_ = controller_parameters_.lam_q_1_.transpose()
      * vehicle_parameters_.inertia_.inverse();

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
                        rrc_control::PlotDataMsg* data_out) {
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
                        rrc_control::PlotDataMsg* data_out) {
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

  Eigen::Vector3d xi_p;
  // xi_p = (position_error.cwiseProduct(position_error)).cwiseSqrt();
  xi_p<<0,0,0;
  
  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  last_time = current_time;

  Eigen::Vector3d dot_hatKp0, dot_hatKp1, dot_rho_p0, dot_rho_p1, delTau_p, zeta_p;
  double dot_hatM;

  if(state_==1){
    dot_hatKp0 = sp.cwiseAbs() - controller_parameters_.alpha_p0_1_.cwiseProduct(controller_parameters_.hatKp0_1_);
    dot_hatKp1 = sp.cwiseAbs().cwiseProduct(xi_p) - controller_parameters_.alpha_p1_1_.cwiseProduct(controller_parameters_.hatKp0_1_);
    dot_hatM = -vehicle_parameters_.gravity_*sp[2] - controller_parameters_.alpha_m_*(controller_parameters_.hatM_1_);
    
    controller_parameters_.hatKp0_1_ += dot_hatKp0*dt;
    controller_parameters_.hatKp1_1_ += dot_hatKp1*dt;  
    controller_parameters_.hatM_1_ += dot_hatM*dt;

    zeta_p = controller_parameters_.hatKp0_1_ + controller_parameters_.rho_p0_1_
                            + (controller_parameters_.hatKp1_1_ + controller_parameters_.rho_p1_1_).cwiseProduct(xi_p);

    delTau_p << zeta_p[0]*Sigmoid(sp[0], controller_parameters_.var_pi_p_), 
  				  zeta_p[1]*Sigmoid(sp[1], controller_parameters_.var_pi_p_), 
  				  zeta_p[2]*Sigmoid(sp[2], controller_parameters_.var_pi_p_);
    
    *thrust =  -controller_parameters_.lam_p_1_.cwiseProduct(sp) - position_error 
                - delTau_p + controller_parameters_.hatM_1_*vehicle_parameters_.gravity_*e_3;
  }
  else if(state_==2){

    dot_hatKp0 = sp.cwiseAbs() - controller_parameters_.alpha_p0_2_.cwiseProduct(controller_parameters_.hatKp0_2_);
    dot_hatKp1 = sp.cwiseAbs().cwiseProduct(xi_p) - controller_parameters_.alpha_p1_2_.cwiseProduct(controller_parameters_.hatKp0_2_);
    dot_hatM = -vehicle_parameters_.gravity_*sp[2] - controller_parameters_.alpha_m_*(controller_parameters_.hatM_2_);

    controller_parameters_.hatKp0_2_ += dot_hatKp0*dt;
    controller_parameters_.hatKp1_2_ += dot_hatKp1*dt;  
    controller_parameters_.hatM_2_ += dot_hatM*dt;

    Eigen::Vector3d zeta_p = controller_parameters_.hatKp0_2_ + controller_parameters_.rho_p0_2_
                            + (controller_parameters_.hatKp1_2_ + controller_parameters_.rho_p1_2_).cwiseProduct(xi_p);

    Eigen::Vector3d delTau_p;
    delTau_p << zeta_p[0]*Sigmoid(sp[0], controller_parameters_.var_pi_p_), 
            zeta_p[1]*Sigmoid(sp[1], controller_parameters_.var_pi_p_), 
            zeta_p[2]*Sigmoid(sp[2], controller_parameters_.var_pi_p_);
    
    *thrust =  -controller_parameters_.lam_p_2_.cwiseProduct(sp) - position_error 
                - delTau_p + controller_parameters_.hatM_2_*vehicle_parameters_.gravity_*e_3;
  }
  else if(state_==3){

    dot_hatKp0 = sp.cwiseAbs() - controller_parameters_.alpha_p0_3_.cwiseProduct(controller_parameters_.hatKp0_3_);
    dot_hatKp1 = sp.cwiseAbs().cwiseProduct(xi_p) - controller_parameters_.alpha_p1_3_.cwiseProduct(controller_parameters_.hatKp0_3_);
    dot_hatM = -vehicle_parameters_.gravity_*sp[2] - controller_parameters_.alpha_m_*(controller_parameters_.hatM_3_);

    controller_parameters_.hatKp0_3_ += dot_hatKp0*dt;
    controller_parameters_.hatKp1_3_ += dot_hatKp1*dt;
    controller_parameters_.hatM_3_ += dot_hatM*dt;

    Eigen::Vector3d zeta_p = controller_parameters_.hatKp0_3_ + controller_parameters_.rho_p0_3_
                            + (controller_parameters_.hatKp1_3_ + controller_parameters_.rho_p1_3_).cwiseProduct(xi_p);

    Eigen::Vector3d delTau_p;
    delTau_p << zeta_p[0]*Sigmoid(sp[0], controller_parameters_.var_pi_p_), 
            zeta_p[1]*Sigmoid(sp[1], controller_parameters_.var_pi_p_), 
            zeta_p[2]*Sigmoid(sp[2], controller_parameters_.var_pi_p_);
    
    *thrust =  -controller_parameters_.lam_p_3_.cwiseProduct(sp) - position_error 
                - delTau_p + controller_parameters_.hatM_3_*vehicle_parameters_.gravity_*e_3;
  }
  else{
    ROS_INFO_STREAM("Invalid state requested. Valid states: 1, 2, 3");
    *thrust << 0, 0, 0;
  }

  if(state_!=1){
    dot_rho_p0 = -(controller_parameters_.beta_p0_1_ 
                  + (varrho_p_1/2)*controller_parameters_.hatKp0_1_.cwiseProduct(controller_parameters_.hatKp0_1_)).cwiseProduct(controller_parameters_.rho_p0_1_)
                  + controller_parameters_.beta_p0_1_.cwiseProduct(controller_parameters_.nu_p0_1_);
    dot_rho_p1 = -(controller_parameters_.beta_p1_1_ 
                  + (varrho_p_1/2)*controller_parameters_.hatKp1_1_.cwiseProduct(controller_parameters_.hatKp1_1_)).cwiseProduct(controller_parameters_.rho_p1_1_)
                  + controller_parameters_.beta_p1_1_.cwiseProduct(controller_parameters_.nu_p1_1_);
    controller_parameters_.rho_p0_1_ += dot_rho_p0*dt;
    controller_parameters_.rho_p1_1_ += dot_rho_p1*dt;
  }
  if(state_!=2){
    dot_rho_p0 = -(controller_parameters_.beta_p0_2_ 
                  + (varrho_p_2/2)*controller_parameters_.hatKp0_2_.cwiseProduct(controller_parameters_.hatKp0_2_)).cwiseProduct(controller_parameters_.rho_p0_2_)
                  + controller_parameters_.beta_p0_2_.cwiseProduct(controller_parameters_.nu_p0_2_);
    dot_rho_p1 = -(controller_parameters_.beta_p1_2_ 
                  + (varrho_p_2/2)*controller_parameters_.hatKp1_2_.cwiseProduct(controller_parameters_.hatKp1_2_)).cwiseProduct(controller_parameters_.rho_p1_2_)
                  + controller_parameters_.beta_p1_2_.cwiseProduct(controller_parameters_.nu_p1_2_);
    controller_parameters_.rho_p0_2_ += dot_rho_p0*dt;
    controller_parameters_.rho_p1_2_ += dot_rho_p1*dt;
  }
  if(state_!=3){
    dot_rho_p0 = -(controller_parameters_.beta_p0_3_ 
                  + (varrho_p_3/2)*controller_parameters_.hatKp0_3_.cwiseProduct(controller_parameters_.hatKp0_3_)).cwiseProduct(controller_parameters_.rho_p0_3_)
                  + controller_parameters_.beta_p0_3_.cwiseProduct(controller_parameters_.nu_p0_3_);
    dot_rho_p1 = -(controller_parameters_.beta_p1_3_ 
                  + (varrho_p_3/2)*controller_parameters_.hatKp1_3_.cwiseProduct(controller_parameters_.hatKp1_3_)).cwiseProduct(controller_parameters_.rho_p1_3_)
                  + controller_parameters_.beta_p1_3_.cwiseProduct(controller_parameters_.nu_p1_3_);
    controller_parameters_.rho_p0_3_ += dot_rho_p0*dt;
    controller_parameters_.rho_p1_3_ += dot_rho_p1*dt;
  }
  // ROS_INFO_STREAM("Force Vector: " << *thrust);
  
  tf::vectorEigenToMsg(position_error, data_out->position_error);
  tf::vectorEigenToMsg(velocity_error, data_out->velocity_error);

  tf::vectorEigenToMsg(controller_parameters_.hatKp0_1_, data_out->Kp1_hat);
  tf::vectorEigenToMsg(controller_parameters_.hatKp0_2_, data_out->Kp2_hat);
  tf::vectorEigenToMsg(controller_parameters_.hatKp0_3_, data_out->Kp3_hat);
  tf::vectorEigenToMsg(controller_parameters_.rho_p0_1_, data_out->p1_rho);
  tf::vectorEigenToMsg(controller_parameters_.rho_p0_2_, data_out->p2_rho);
  tf::vectorEigenToMsg(controller_parameters_.rho_p0_3_, data_out->p3_rho);
  tf::vectorEigenToMsg(sp, data_out->sp);
  tf::vectorEigenToMsg(zeta_p, data_out->zeta_p);
  tf::vectorEigenToMsg(delTau_p, data_out->delTau_p);

  data_out->M1_hat = controller_parameters_.hatM_1_;
  data_out->M2_hat = controller_parameters_.hatM_2_;
  data_out->M3_hat = controller_parameters_.hatM_3_;
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void SacPositionController::CalculateMoments(Eigen::Vector3d force, 
												Eigen::Vector3d* moments, 
                        rrc_control::PlotDataMsg* data_out) {
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

	  Eigen::Vector3d xq_norm;
	  xq_norm = angle_error.cwiseAbs();

	  // ROS_INFO_STREAM("XQ_NORM" << xq_norm);

	  Eigen::Vector3d sq_norm = sq.cwiseAbs();
	  // Eigen::Vector3d sq_norm = angle_error.cwiseAbs();

	  Eigen::Vector3d xqx_norm, xqy_norm, xqz_norm;
	  xqx_norm << 1, xq_norm[0], pow(xq_norm[0],2);
	  xqy_norm << 1, xq_norm[1], pow(xq_norm[1],2);
	  xqz_norm << 1, xq_norm[2], pow(xq_norm[2],2);
	  
    current_time = ros::Time::now();
	  double dt = (current_time - last_time).toSec();
    last_time = current_time;
	  
    Eigen::Vector3d zeta_q;
	  Eigen::Vector3d delTau_q;
	  Eigen::Vector3d dot_hatKq0, dot_hatKq1, dot_hatKq2, dot_rho_q0, dot_rho_q1, dot_rho_q2;

	  // ROS_INFO_STREAM("XQXYZ/ done" << xqx_norm);
    if(state_==1){

  	  dot_hatKq0 = sq_norm[0]*xqx_norm - controller_parameters_.alpha_q0_1_.cwiseProduct(controller_parameters_.hatKq0_1_);
  	  dot_hatKq1 = sq_norm[1]*xqy_norm - controller_parameters_.alpha_q1_1_.cwiseProduct(controller_parameters_.hatKq1_1_);
  	  dot_hatKq2 = sq_norm[2]*xqz_norm - controller_parameters_.alpha_q2_1_.cwiseProduct(controller_parameters_.hatKq2_1_);
  	  
  	  controller_parameters_.hatKq0_1_ += dot_hatKq0*dt;
      controller_parameters_.hatKq1_1_ += dot_hatKq1*dt;
      controller_parameters_.hatKq2_1_ += dot_hatKq2*dt;

  	  zeta_q[0] = (controller_parameters_.hatKq0_1_ + controller_parameters_.rho_q0_1_).dot(xqx_norm);
      zeta_q[1] = (controller_parameters_.hatKq1_1_ + controller_parameters_.rho_q1_1_).dot(xqy_norm);
      zeta_q[2] = (controller_parameters_.hatKq2_1_ + controller_parameters_.rho_q2_1_).dot(xqz_norm);

  	  // ROS_INFO("RHo done");


  	  // delTau_q << rho_q[0]*Sigmoid(sq[0]), rho_q[1]*Sigmoid(sq[1]), rho_q[2]*Sigmoid(sq[2]);
  	  delTau_q << zeta_q[0]*Sigmoid(sq[0], controller_parameters_.var_pi_q_), 
  				  	zeta_q[1]*Sigmoid(sq[1], controller_parameters_.var_pi_q_), 
  				  	zeta_q[2]*Sigmoid(sq[2], controller_parameters_.var_pi_q_);

  	  // ROS_INFO_STREAM(delTau_q);

  	  *moments = -controller_parameters_.lam_q_1_.cwiseProduct(sq) - delTau_q;
    }
    else if(state_==2){

      dot_hatKq0 = sq_norm[0]*xqx_norm - controller_parameters_.alpha_q0_2_.cwiseProduct(controller_parameters_.hatKq0_2_);
      dot_hatKq1 = sq_norm[1]*xqy_norm - controller_parameters_.alpha_q1_2_.cwiseProduct(controller_parameters_.hatKq1_2_);
      dot_hatKq2 = sq_norm[2]*xqz_norm - controller_parameters_.alpha_q2_2_.cwiseProduct(controller_parameters_.hatKq2_2_);

      
      controller_parameters_.hatKq0_2_ += dot_hatKq0*dt;
      controller_parameters_.hatKq1_2_ += dot_hatKq1*dt;
      controller_parameters_.hatKq2_2_ += dot_hatKq2*dt;
      

      zeta_q[0] = (controller_parameters_.hatKq0_2_ + controller_parameters_.rho_q0_2_).dot(xqx_norm);
      zeta_q[1] = (controller_parameters_.hatKq1_2_ + controller_parameters_.rho_q1_2_).dot(xqy_norm);
      zeta_q[2] = (controller_parameters_.hatKq2_2_ + controller_parameters_.rho_q2_2_).dot(xqz_norm);

      // ROS_INFO("RHo done");


      // delTau_q << rho_q[0]*Sigmoid(sq[0]), rho_q[1]*Sigmoid(sq[1]), rho_q[2]*Sigmoid(sq[2]);
      delTau_q << zeta_q[0]*Sigmoid(sq[0], controller_parameters_.var_pi_q_), 
              zeta_q[1]*Sigmoid(sq[1], controller_parameters_.var_pi_q_), 
              zeta_q[2]*Sigmoid(sq[2], controller_parameters_.var_pi_q_);

      // ROS_INFO_STREAM(delTau_q);

      *moments = -controller_parameters_.lam_q_2_.cwiseProduct(sq) - delTau_q;
    }
    else if(state_==3){

      dot_hatKq0 = sq_norm[0]*xqx_norm - controller_parameters_.alpha_q0_3_.cwiseProduct(controller_parameters_.hatKq0_3_);
      dot_hatKq1 = sq_norm[1]*xqy_norm - controller_parameters_.alpha_q1_3_.cwiseProduct(controller_parameters_.hatKq1_3_);
      dot_hatKq2 = sq_norm[2]*xqz_norm - controller_parameters_.alpha_q2_3_.cwiseProduct(controller_parameters_.hatKq2_3_);

      
      controller_parameters_.hatKq0_3_ += dot_hatKq0*dt;
      controller_parameters_.hatKq1_3_ += dot_hatKq1*dt;
      controller_parameters_.hatKq2_3_ += dot_hatKq2*dt;
      

      zeta_q[0] = (controller_parameters_.hatKq0_3_ + controller_parameters_.rho_q0_3_).dot(xqx_norm);
      zeta_q[1] = (controller_parameters_.hatKq1_3_ + controller_parameters_.rho_q1_3_).dot(xqy_norm);
      zeta_q[2] = (controller_parameters_.hatKq2_3_ + controller_parameters_.rho_q2_3_).dot(xqz_norm);

      // ROS_INFO("RHo done");


      // delTau_q << rho_q[0]*Sigmoid(sq[0]), rho_q[1]*Sigmoid(sq[1]), rho_q[2]*Sigmoid(sq[2]);
      delTau_q << zeta_q[0]*Sigmoid(sq[0], controller_parameters_.var_pi_q_), 
              zeta_q[1]*Sigmoid(sq[1], controller_parameters_.var_pi_q_), 
              zeta_q[2]*Sigmoid(sq[2], controller_parameters_.var_pi_q_);

      // ROS_INFO_STREAM(delTau_q);

      *moments = -controller_parameters_.lam_q_3_.cwiseProduct(sq) - delTau_q;
    }
    else{
      *moments<<0,0,0;
    }

    if(state_!=1){
      dot_rho_q0 = -(controller_parameters_.beta_q0_1_ 
                    + (varrho_q_1/2)*controller_parameters_.hatKq0_1_.cwiseProduct(controller_parameters_.hatKq0_1_)).cwiseProduct(controller_parameters_.rho_q0_1_)
                    + controller_parameters_.beta_q0_1_.cwiseProduct(controller_parameters_.nu_q0_1_);
      dot_rho_q1 = -(controller_parameters_.beta_q1_1_ 
                    + (varrho_q_1/2)*controller_parameters_.hatKq1_1_.cwiseProduct(controller_parameters_.hatKq1_1_)).cwiseProduct(controller_parameters_.rho_q1_1_)
                    + controller_parameters_.beta_q1_1_.cwiseProduct(controller_parameters_.nu_q1_1_);
      dot_rho_q2 = -(controller_parameters_.beta_q2_1_ 
                    + (varrho_q_1/2)*controller_parameters_.hatKq2_1_.cwiseProduct(controller_parameters_.hatKq2_1_)).cwiseProduct(controller_parameters_.rho_q2_1_)
                    + controller_parameters_.beta_q2_1_.cwiseProduct(controller_parameters_.nu_q2_1_);
      controller_parameters_.rho_q0_1_ += dot_rho_q0*dt;
      controller_parameters_.rho_q1_1_ += dot_rho_q1*dt;
      controller_parameters_.rho_q2_1_ += dot_rho_q2*dt;
    }
    if (state_!=2){
      dot_rho_q0 = -(controller_parameters_.beta_q0_2_ 
                    + (varrho_q_2/2)*controller_parameters_.hatKq0_2_.cwiseProduct(controller_parameters_.hatKq0_2_)).cwiseProduct(controller_parameters_.rho_q0_2_)
                    + controller_parameters_.beta_q0_2_.cwiseProduct(controller_parameters_.nu_q0_2_);
      dot_rho_q1 = -(controller_parameters_.beta_q1_2_ 
                    + (varrho_q_2/2)*controller_parameters_.hatKq1_2_.cwiseProduct(controller_parameters_.hatKq1_2_)).cwiseProduct(controller_parameters_.rho_q1_2_)
                    + controller_parameters_.beta_q1_2_.cwiseProduct(controller_parameters_.nu_q1_2_);
      dot_rho_q2 = -(controller_parameters_.beta_q2_2_ 
                    + (varrho_q_2/2)*controller_parameters_.hatKq2_2_.cwiseProduct(controller_parameters_.hatKq2_2_)).cwiseProduct(controller_parameters_.rho_q2_2_)
                    + controller_parameters_.beta_q2_2_.cwiseProduct(controller_parameters_.nu_q2_2_);
      controller_parameters_.rho_q0_2_ += dot_rho_q0*dt;
      controller_parameters_.rho_q1_2_ += dot_rho_q1*dt;
      controller_parameters_.rho_q2_2_ += dot_rho_q2*dt;
    }
    if (state_!=3){
      dot_rho_q0 = -(controller_parameters_.beta_q0_3_ 
                    + (varrho_q_3/2)*controller_parameters_.hatKq0_3_.cwiseProduct(controller_parameters_.hatKq0_3_)).cwiseProduct(controller_parameters_.rho_q0_3_)
                    + controller_parameters_.beta_q0_3_.cwiseProduct(controller_parameters_.nu_q0_3_);
      dot_rho_q1 = -(controller_parameters_.beta_q1_3_ 
                    + (varrho_q_3/2)*controller_parameters_.hatKq1_3_.cwiseProduct(controller_parameters_.hatKq1_3_)).cwiseProduct(controller_parameters_.rho_q1_3_)
                    + controller_parameters_.beta_q1_3_.cwiseProduct(controller_parameters_.nu_q1_3_);
      dot_rho_q2 = -(controller_parameters_.beta_q2_3_ 
                    + (varrho_q_3/2)*controller_parameters_.hatKq2_3_.cwiseProduct(controller_parameters_.hatKq2_3_)).cwiseProduct(controller_parameters_.rho_q2_3_)
                    + controller_parameters_.beta_q2_3_.cwiseProduct(controller_parameters_.nu_q2_3_);
      controller_parameters_.rho_q0_3_ += dot_rho_q0*dt;
      controller_parameters_.rho_q1_3_ += dot_rho_q1*dt;
      controller_parameters_.rho_q2_3_ += dot_rho_q2*dt;
    }
    // *moments = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
    //                        - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
    //                        + odometry_.angular_velocity.cross(odometry_.angular_velocity);

    // *moments = vehicle_parameters_.inertia_ * *moments;

    tf::vectorEigenToMsg(angle_error, data_out->angle_error);
    tf::vectorEigenToMsg(angular_rate_error, data_out->angle_rate_error);

    tf::vectorEigenToMsg(controller_parameters_.hatKq0_1_, data_out->Kq1_hat_0);
    tf::vectorEigenToMsg(controller_parameters_.hatKq1_1_, data_out->Kq1_hat_1);
    tf::vectorEigenToMsg(controller_parameters_.hatKq2_1_, data_out->Kq1_hat_2);
    tf::vectorEigenToMsg(controller_parameters_.hatKq0_2_, data_out->Kq2_hat_0);
    tf::vectorEigenToMsg(controller_parameters_.hatKq1_2_, data_out->Kq2_hat_1);
    tf::vectorEigenToMsg(controller_parameters_.hatKq2_2_, data_out->Kq2_hat_2);
    tf::vectorEigenToMsg(controller_parameters_.hatKq0_3_, data_out->Kq3_hat_0);
    tf::vectorEigenToMsg(controller_parameters_.hatKq1_3_, data_out->Kq3_hat_1);
    tf::vectorEigenToMsg(controller_parameters_.hatKq2_3_, data_out->Kq3_hat_2);
    tf::vectorEigenToMsg(controller_parameters_.rho_q0_1_, data_out->q1_rho_0);
    tf::vectorEigenToMsg(controller_parameters_.rho_q1_1_, data_out->q1_rho_1);
    tf::vectorEigenToMsg(controller_parameters_.rho_q2_1_, data_out->q1_rho_2);
    tf::vectorEigenToMsg(controller_parameters_.rho_q0_2_, data_out->q2_rho_0);
    tf::vectorEigenToMsg(controller_parameters_.rho_q1_2_, data_out->q2_rho_1);
    tf::vectorEigenToMsg(controller_parameters_.rho_q2_2_, data_out->q2_rho_2);
    tf::vectorEigenToMsg(controller_parameters_.rho_q0_3_, data_out->q3_rho_0);
    tf::vectorEigenToMsg(controller_parameters_.rho_q1_3_, data_out->q3_rho_1);
    tf::vectorEigenToMsg(controller_parameters_.rho_q2_3_, data_out->q3_rho_2);
    tf::vectorEigenToMsg(sq, data_out->sq);
    tf::vectorEigenToMsg(zeta_q, data_out->zeta_q);
    tf::vectorEigenToMsg(delTau_q, data_out->delTau_q);

  }
}


inline double SacPositionController::Sigmoid(double s, double var_pi) const {
	return ((abs(s) > var_pi) ? s/abs(s) : s/var_pi);
}

inline double SacPositionController::Minimum(double x, double y) const {
  return ((x > y) ? x : y);
}



}
