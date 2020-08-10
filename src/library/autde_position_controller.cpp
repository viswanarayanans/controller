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

#include "rrc_control/autde_position_controller.h"
#include <eigen_conversions/eigen_msg.h>


namespace rrc_control {

AuTdePositionController::AuTdePositionController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

AuTdePositionController::~AuTdePositionController() {}

void AuTdePositionController::InitializeParameters() {
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  // To make the tuning independent of the inertia matrix we divide here.
  // normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose()
      // * vehicle_parameters_.inertia_.inverse();
  // To make the tuning independent of the inertia matrix we divide here.
  // normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose()
      // * vehicle_parameters_.inertia_.inverse();

  //Added by Viswa : To add payload mass with base mass
  // normalized_mass = vehicle_parameters_.mass_ + vehicle_parameters_.payload_mass_;
  

  // Eigen::Matrix4d I;
  // I.setZero();
  // I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
  // I(3, 3) = 1;
  // I.setIdentity();



  //ROS_INFO("Inertia : %f %f %f", vehicle_parameters_.inertia_(0,0), vehicle_parameters_.inertia_(1,1), vehicle_parameters_.inertia_(2,2));

  rotor_vel_coef_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  rotor_vel_coef_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse();
  initialized_params_ = true;
}

void AuTdePositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities, 
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

  static Eigen::Vector3d thrust_3d(0,0,0);
  CalculateThrust(&thrust_3d, data_out);
  // ROS_INFO_STREAM("Thrust done"<<thrust_3d);
  if(thrust_3d[2]<-1){
    thrust_3d[2] = -1;
  }

  double thrust = thrust_3d.dot(odometry_.orientation.toRotationMatrix().col(2)); //Added by Viswa
  ROS_INFO_STREAM("Thrust:"<<thrust);
  data_out->thrust = thrust;


  static Eigen::Vector3d moments(0,0,0);
  CalculateMoments(thrust_3d, &moments, data_out);
  tf::vectorEigenToMsg(moments, data_out->moments);
  ROS_INFO_STREAM("Moments:"<<moments);


  // Project thrust onto body z axis.

  // const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix(); //Added by Viswa
  // double velocity_W =  (R_W_I * odometry_.velocity).z(); //Added by Viswa
  
  // double position_error; //Added by Viswa
  // position_error = odometry_.position.z() - command_trajectory_.position_W.z(); //Added by Viswa


  Eigen::Vector4d moment_thrust;
  moment_thrust.block<3, 1>(0, 0) = moments;
  moment_thrust(3) = thrust;

  *rotor_velocities = rotor_vel_coef_ * moment_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}

void AuTdePositionController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void AuTdePositionController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void AuTdePositionController::CalculateThrust(Eigen::Vector3d* thrust, 
                        msg_check::PlotDataMsg* data_out) const {
  assert(thrust);

  Eigen::Vector3d position_error;
  position_error = odometry_.position - command_trajectory_.position_W;

  const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
  Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
  Eigen::Vector3d velocity_error;
  velocity_error = velocity_W - command_trajectory_.velocity_W;

  Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());


  static ros::Time last_time = ros::Time::now();
  ros::Time current_time;

  static Eigen::Vector3d previous_velocity(0,0,0);
  
  Eigen::VectorXd xp(6);
  xp << position_error, velocity_error;

  Eigen::Vector3d sp;
  sp = velocity_error + controller_parameters_.theta_p_.cwiseProduct(position_error);

  static double hatKp_0 = controller_parameters_.hatKp_0_;
  static double hatKp_1 = controller_parameters_.hatKp_1_;
  static double hatM = controller_parameters_.hatM_;
  static double gamma_p = controller_parameters_.gamma_p_;

  double dot_hatKp_0, dot_hatKp_1;
  double dot_hatM;
  double dot_gamma_p;


  dot_hatKp_0 = sp.norm() - controller_parameters_.alpha_p0_*hatKp_0;
  dot_hatKp_1 = sp.norm()*xp.norm() - controller_parameters_.alpha_p1_*hatKp_1;
  dot_gamma_p = -(1 + pow(xp.norm(),4)) + 0.001;
  dot_hatM = -vehicle_parameters_.gravity_*sp[2] - controller_parameters_.alpha_m_*(hatM);


  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();

  if(dt>0 && dt<1){
  
    hatKp_0 += dot_hatKp_0*dt;
    hatKp_1 += dot_hatKp_1*dt;
    hatM += dot_hatM*dt;
    ROS_INFO_STREAM("Mass:"<<hatM);
    gamma_p += dot_gamma_p*dt;


    // double rho_p = hatKp_0 + hatKp_1*xp.norm() + gamma_p;
    double rho_p = hatKp_0 + hatKp_1*xp.norm();

    Eigen::Vector3d sgp;
    sgp << Sigmoid(sp[0], controller_parameters_.var_pi_p_), Sigmoid(sp[1], controller_parameters_.var_pi_p_), Sigmoid(sp[2], controller_parameters_.var_pi_p_);

    Eigen::Vector3d v_p = command_trajectory_.acceleration_W - controller_parameters_.Kpp_.cwiseProduct(position_error)
          + controller_parameters_.Kdp_.cwiseProduct(velocity_error) - rho_p*sgp + hatM*vehicle_parameters_.gravity_*e_3;
    ROS_INFO_STREAM("Vp: "<<v_p);  

    Eigen::Vector3d accel = (odometry_.velocity - previous_velocity)/dt;
    ROS_INFO_STREAM("Position:"<<odometry_.position[2]);
    ROS_INFO_STREAM("Accel:"<<accel);
    ROS_INFO_STREAM("Dt:"<<dt);

    // ROS_INFO_STREAM("Up thrust:"<<*thrust);

    *thrust = vehicle_parameters_.mass_*v_p + *thrust;
    *thrust = *thrust - vehicle_parameters_.mass_*accel;

    // ROS_INFO_STREAM("Thrust Up:"<<*thrust);

    // *thrust = *thrust - vehicle_parameters_.mass_*(odometry_.velocity - previous_velocity)/dt;
  }

  last_time = current_time;
  previous_velocity = odometry_.velocity;
  
  // tf::vectorEigenToMsg(*acceleration, data_out->acceleration);
  tf::vectorEigenToMsg(position_error, data_out->position_error);
  tf::vectorEigenToMsg(velocity_error, data_out->velocity_error);

  // tf::vectorEigenToMsg(hatKp, data_out->Kp_hat);
  tf::vectorEigenToMsg(sp, data_out->sp);
  // tf::vectorEigenToMsg(rho_p, data_out->rho_p);
  // tf::vectorEigenToMsg(delTau_p, data_out->delTau_p);

  data_out->M_hat = hatM;
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void AuTdePositionController::CalculateMoments(Eigen::Vector3d force, 
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

    static Eigen::Vector3d previous_ang_velocity(0,0,0);

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

    static double hatKq_0 = controller_parameters_.hatKq_0_;
    static double hatKq_1 = controller_parameters_.hatKq_1_;
    static double gamma_q = controller_parameters_.gamma_q_;

    double dot_hatKq_0, dot_hatKq_1;
    double dot_gamma_q;

    Eigen::VectorXd xq(6);
    xq << angle_error, angular_rate_error;

    dot_hatKq_0 = sq.norm() - controller_parameters_.alpha_q0_*hatKq_0;
    dot_hatKq_1 = sq.norm()*xq.norm() - controller_parameters_.alpha_q1_*hatKq_1;
    dot_gamma_q = -(1 + pow(xq.norm(),4)) + 0.001;
	  

	  // ROS_INFO_STREAM("XQ_NORM" << xq_norm);

	  current_time = ros::Time::now();
	  double dt = (current_time - last_time).toSec();
    
    if (dt>0 && dt<1){

      hatKq_0 += dot_hatKq_0*dt;
      hatKq_1 += dot_hatKq_1*dt;
      gamma_q += dot_gamma_q*dt;
  	  
  	  // ROS_INFO("hatKq done");

  	  // Eigen::Matrix3d lam_q = controller_parameters_.lam_q_.asDiagonal();
      double rho_q = hatKq_0 + hatKq_1*xq.norm() + gamma_q;
      Eigen::Vector3d sgq;
      sgq << Sigmoid(sq[0], controller_parameters_.var_pi_q_), Sigmoid(sq[1], controller_parameters_.var_pi_q_), Sigmoid(sq[2], controller_parameters_.var_pi_q_);
      // Eigen::Vector3d v_q = -controller_parameters_.Kpq_.cwiseProduct(angle_error) - controller_parameters_.Kdq_.cwiseProduct(angular_rate_error) - rho_q*sgq;
      Eigen::Vector3d v_q = -controller_parameters_.Kpq_.cwiseProduct(angle_error) - controller_parameters_.Kdq_.cwiseProduct(angular_rate_error) ;

      // *moments = vehicle_parameters_.inertia_* v_q + *moments;
      *moments = vehicle_parameters_.inertia_* v_q ;

      Eigen::Vector3d ang_acc = (odometry_.angular_velocity - previous_ang_velocity)/dt;
      ROS_INFO_STREAM("Ang Acc:"<<ang_acc);

      // *moments = *moments - vehicle_parameters_.inertia_*ang_acc;
    }

    last_time = current_time;
    previous_ang_velocity = odometry_.angular_velocity;

	  // *moments = -controller_parameters_.lam_q_.cwiseProduct(sq) - delTau_q;
    // tf::vectorEigenToMsg(*angular_acceleration, data_out->angular_acceleration);
    tf::vectorEigenToMsg(angle_error, data_out->angle_error);
    tf::vectorEigenToMsg(angular_rate_error, data_out->angle_rate_error);

    // tf::vectorEigenToMsg(hatKq_0, data_out->Kq_hat_0);
    // tf::vectorEigenToMsg(hatKq_1, data_out->Kq_hat_1);
    // tf::vectorEigenToMsg(hatKq_2, data_out->Kq_hat_2);
    // tf::vectorEigenToMsg(sq, data_out->sq);
    // tf::vectorEigenToMsg(rho_q, data_out->rho_q);
    // tf::vectorEigenToMsg(delTau_q, data_out->delTau_q);


	  // *moments = -controller_parameters_.lam_q_.cwiseProduct(sq) ;
	  // ROS_INFO_STREAM(*moments);
  }
	  // ROS_INFO("Attitude"	);
  // *moments = Eigen::Vector3d(0,0,0);

  // ROS_INFO_STREAM("Moments"<< *moments);

  

  // *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
  //                          - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
  //                          + odometry_.angular_velocity.cross(odometry_.angular_velocity); // we don't need the inertia matrix here
}

// void SmcPositionController::Sigmoid(double s, double* sig) const{
// 	if (s > controller_parameters_.var_pi_) {
// 		*sig =  s/abs(s);
// 	}
// 	else {
// 		*sig =  s/controller_parameters_.var_pi_;
// 	}
// }

/*double SmcPositionController::Sigmoid(double s) const {
	// if (s > controller_parameters_.var_pi_) {
	// 	return  s/abs(s);
	// }
	// else {
	// 	return  s/controller_parameters_.var_pi_;
	// }
	return ((abs(s) > controller_parameters_.var_pi_p_) ? s/abs(s) : s/controller_parameters_.var_pi_p_);
}*/

double AuTdePositionController::Sigmoid(double s, double var_pi) const {
	return ((abs(s) > var_pi) ? s/abs(s) : s/var_pi);
}


}


// void callback(const sensor_msgs::LaserScan::ConstPtr& input) {
  //ROS_INFO("Laser Signal Recieved");
// }

