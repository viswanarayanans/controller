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

#include "viswa_control/asmc_position_controller.h"

namespace viswa_control {

ASmcPositionController::ASmcPositionController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

ASmcPositionController::~ASmcPositionController() {}

void ASmcPositionController::InitializeParameters() {
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

void ASmcPositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const {
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
  CalculateThrust(&thrust_3d);
  // ROS_INFO("Thrust done");

  double thrust = thrust_3d.dot(odometry_.orientation.toRotationMatrix().col(2)); //Added by Viswa
  ROS_INFO_STREAM(thrust);

  Eigen::Vector3d moments;
  CalculateMoments(thrust_3d, &moments);

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

  // Eigen::Matrix4d I;
  // I.setZero();
  // I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
  // I(3, 3) = 1;
  // static bool control_flag=0;

  // if((odometry_.velocity.z() > 0.01) || (odometry_.velocity.z() < -0.01))
  // {
  //   control_flag = 0;
  //   //ROS_INFO("Control Flag is ON, %f", acceleration.z());

  // }

 
  // if((odometry_.velocity.z() < 0.0001) && control_flag && (odometry_.velocity.z() > -0.0001))
  // {
  //   if ((position_error < -0.1))
  //   {
  //     double payload_mass = - (controller_parameters_.position_gain_.z() * (position_error))/vehicle_parameters_.gravity_;
  //     normalized_mass = vehicle_parameters_.mass_ + payload_mass;
  //     //UpdateMassAndInertia(new_mass);      
  //     ROS_INFO("%f Normalized Mass: %f", odometry_.position.z(), payload_mass);
  //     control_flag = 0;
  //     ROS_INFO("%f, %f", acceleration.z(), odometry_.velocity.z());
  //   }
  //     integral_active_ = 1;
  //     //ROS_INFO("Integral gain active");
  // }
  
  //ROS_INFO("Feeding Calculated motor speeds");

  
  /*double test_mass = ((I.inverse() *
                  rotor_vel_coef_ * (*rotor_velocities))[3] +
                  controller_parameters_.position_gain_.z() * position_error +
                  controller_parameters_.velocity_gain_.z() *
                  velocity_W)/vehicle_parameters_.gravity_; //Added by Viswa
  
*/
  /*
  double test_mass = (thrust + controller_parameters_.position_gain_.z() *
                  position_error + controller_parameters_.velocity_gain_.z() *
                  velocity_W)/vehicle_parameters_.gravity_ ; //Added by Viswa
  */
  /*ROS_INFO("Tested mass is: %f", test_mass); //Added by Viswa*/
}

/*void LeePositionController::UpdateMassAndInertia(double new_mass) 
{
  vehicle_parameters_.payload_mass_ = new_mass;
  normalized_mass = vehicle_parameters_.mass_ + vehicle_parameters_.payload_mass_;
}*/

void ASmcPositionController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void ASmcPositionController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void ASmcPositionController::CalculateThrust(Eigen::Vector3d* thrust) const {
  assert(thrust);

  Eigen::Vector3d position_error;
  position_error = odometry_.position - command_trajectory_.position_W;

  // Transform velocity to world frame.
  const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
  Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
  Eigen::Vector3d velocity_error;
  velocity_error = velocity_W - command_trajectory_.velocity_W;

  Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());


  // position_error_integral = position_error_integral + position_error;
  static ros::Time last_time = ros::Time::now();
  ros::Time current_time;
  
  // Eigen::VectorXd xp(6);
  // xp << position_error, velocity_error;

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
  // Eigen::Vector3d sat(Sigmoid(sp[0]), Sigmoid(sp[1]), Sigmoid(sp[2]));

  // delTau_p << rho_p[0]*Sigmoid(sp[0]), rho_p[1]*Sigmoid(sp[1]), rho_p[2]*Sigmoid(sp[2]);
  delTau_p << rho_p[0]*Sigmoid(sp[0], controller_parameters_.var_pi_p_), 
				  rho_p[1]*Sigmoid(sp[1], controller_parameters_.var_pi_p_), 
				  rho_p[2]*Sigmoid(sp[2], controller_parameters_.var_pi_p_);
  // ROS_INFO_STREAM(delTau_p);

  // Sigmoid(sp[0], &sat[0]);
  // Sigmoid(sp[1], &sat[1]);
  // Sigmoid(sp[2], &sat[2]);

  // delTau_p << 0, 0, 0;

  *thrust =  -lam_p*sp - delTau_p + hatM*vehicle_parameters_.gravity_*e_3;

  // ROS_INFO_STREAM(*thrust);
  // *thrust = -lam_p*sp + Eigen::Vector3d(0,0,19.6);

  // ROS_INFO_STREAM("Thrust: "<< *thrust);



  // *acceleration = (position_error.cwiseProduct(controller_parameters_.position_gain_)
  //   + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)
  //   + position_error_integral.cwiseProduct(controller_parameters_.position_integral_gain_)) / normalized_mass
  //   - vehicle_parameters_.gravity_ * e_3 - command_trajectory_.acceleration_W; //Added by Viswa


  //ROS_INFO("Mass : %f", normalized_mass);

  //ROS_INFO("Mass of Iris: %f, %f", vehicle_parameters_.mass_, vehicle_parameters_.payload_mass_); //Added by Viswa
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void ASmcPositionController::CalculateMoments(Eigen::Vector3d force, 
												Eigen::Vector3d* moments) const {
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

	  // Eigen::Vector3d sq;
	  // sq = angular_rate_error + controller_parameters_.theta_q_*angle_error;
	  // Eigen::Matrix3d lam_q = controller_parameters_.lam_q_.asDiagonal();

	  // Eigen::Vector3d tau_q = -(lam_q*vehicle_parameters_.inertia_.inverse())*sq + 0.00*odometry_.angular_velocity.cross(odometry_.angular_velocity);
	  // Eigen::Vector3d normalized_angle_gain_ = Eigen::Vector3d(1,1,0.035).transpose()*vehicle_parameters_.inertia_.inverse();
	  // Eigen::Vector3d normalized_ang_rate_gain_ = Eigen::Vector3d(0.22, 0.22, 0.01).transpose()*vehicle_parameters_.inertia_.inverse();

	  /*Eigen::VectorXd xqx(2), xqy(2), xqz(2);
	  xqx << angle_error[0], angular_rate_error[0];
	  xqy << angle_error[1], angular_rate_error[1];
	  xqz << angle_error[2], angular_rate_error[2];*/

	  // ROS_INFO_STREAM("XQ" << xqx);

	  Eigen::Vector3d sq;
	  sq = angular_rate_error + controller_parameters_.theta_q_.cwiseProduct(angle_error);
	  
	  // ROS_INFO_STREAM(xqx, xqy);

	  Eigen::Vector3d dot_hatKq_0, dot_hatKq_1, dot_hatKq_2;
	  Eigen::Vector3d xq_norm;
	  /*xq_norm << xqx.norm(), xqy.norm(), xqz.norm();*/
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

	  // ROS_INFO("dot_hatKq done");


	  // dot_hatKq[0] = sq_norm - controller_parameters_.alpha_[0]*hatKq[0];
	  // dot_hatKq[1] = sq_norm*xq_norm - controller_parameters_.alpha_[1]*hatKq[1];
	  // dot_hatKq[2] = sq_norm*pow(xq_norm,2) - controller_parameters_.alpha_[2]*hatKq[2];

	  current_time = ros::Time::now();
	  double dt = (current_time - last_time).toSec();
	  
	  hatKq_0 += dot_hatKq_0*dt;
	  // hatKq_0[2] = 0;


	  hatKq_1 += dot_hatKq_1*dt;
	  // hatKq_1[2] = 0;

	  hatKq_2 += dot_hatKq_2*dt;
	  // hatKq_2[2] = 0;

	  // ROS_INFO_STREAM(hatKq_0);



	  // hatKq += dot_hatKq;    
	  
	  last_time = current_time;
	  // ROS_INFO("hatKq done");

	  // Eigen::Matrix3d lam_q = controller_parameters_.lam_q_.asDiagonal();
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

	  ROS_INFO_STREAM(delTau_q);
	  // ROS_INFO("Del_tau done");
		// 
	  // Eigen::Vector3d tau_q = -lam_q*sq - delTau_q;
	    

	  // Eigen::Vector3d tau_q = -angle_error.cwiseProduct(normalized_angle_gain_) - angular_rate_error.cwiseProduct(normalized_ang_rate_gain_)  + 0*odometry_.angular_velocity.cross(odometry_.angular_velocity);
	  // Eigen::Vector3d tau_q = -4.5*angle_error.cwiseProduct(controller_parameters_.lam_q_) - angular_rate_error.cwiseProduct(controller_parameters_.lam_q_);
	  // ROS_INFO_STREAM(lam_q);

	  *moments = -controller_parameters_.lam_q_.cwiseProduct(sq) - delTau_q;

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

double ASmcPositionController::Sigmoid(double s, double var_pi) const {
	return ((abs(s) > var_pi) ? s/abs(s) : s/var_pi);
}



}


// void callback(const sensor_msgs::LaserScan::ConstPtr& input) {
  //ROS_INFO("Laser Signal Recieved");
// }

