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

#include "rrc_control/reconfig_pid_controller.h"
#include <eigen_conversions/eigen_msg.h>

namespace rrc_control {

ReconfigPidController::ReconfigPidController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

ReconfigPidController::~ReconfigPidController() {}

void ReconfigPidController::InitializeParameters() {
  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();

  //Added by Viswa : To add payload mass with base mass
  normalized_mass = vehicle_parameters_.mass_ + vehicle_parameters_.payload_mass_;


  

  Eigen::Matrix4d I;
  I.setIdentity();
  I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
  I(3, 3) = 1;



  //ROS_INFO("Inertia : %f %f %f", vehicle_parameters_.inertia_(0,0), vehicle_parameters_.inertia_(1,1), vehicle_parameters_.inertia_(2,2));
  
  angular_acc_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;
  
  // ROS_INFO_STREAM("Gains \n" << normalized_attitude_gain_ << "\n" << normalized_angular_rate_gain_);
  // ROS_INFO_STREAM("allocation_matrix_\n" << controller_parameters_.allocation_matrix_);
  // ROS_INFO_STREAM("I \n" << I);
  ROS_INFO_STREAM("angular_acc_to_rotor_velocities_\n" << angular_acc_to_rotor_velocities_);
  // ROS_INFO_STREAM("\n"<< (controller_parameters_.allocation_matrix_
  //     * controller_parameters_.allocation_matrix_.transpose()));
  // ROS_INFO_STREAM("\n"<< controller_parameters_.allocation_matrix_.transpose()
  //     * (controller_parameters_.allocation_matrix_
  //     * controller_parameters_.allocation_matrix_.transpose()).inverse());

  initialized_params_ = true;
}

void ReconfigPidController::ResetParameters(double box_mass, Eigen::Vector3d* box_dim) {
  Eigen::Vector3d dim = *box_dim;

  Eigen::Vector3d box_inertia;
  box_inertia[0] = box_mass*(dim[1]*dim[1] + dim[2]*dim[2])/12;
  box_inertia[1] = box_mass*(dim[0]*dim[0] + dim[2]*dim[2])/12;
  box_inertia[2] = box_mass*(dim[0]*dim[0] + dim[1]*dim[1])/12;

  Eigen::Vector3d rotor_inertia;
  rotor_inertia[0] = 0.08*(((dim[0]/2)+14.142136)*((dim[0]/2)+14.142136));
  rotor_inertia[1] = 0.08*(((dim[1]/2)+14.142136)*((dim[1]/2)+14.142136));
  rotor_inertia[2] = rotor_inertia[0] + rotor_inertia[1];

  vehicle_parameters_.inertia_(0, 0) = box_inertia[0] + rotor_inertia[0];
  vehicle_parameters_.inertia_(1, 1) = box_inertia[1] + rotor_inertia[1];
  vehicle_parameters_.inertia_(2, 2) = box_inertia[2] + rotor_inertia[2];


  // To make the tuning independent of the inertia matrix we divide here.
  normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();

  //Added by Viswa : To add payload mass with base mass
  normalized_mass = vehicle_parameters_.mass_ + vehicle_parameters_.payload_mass_;


  

  Eigen::Matrix4d I;
  I.setIdentity();
  I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
  I(3, 3) = 1;



  //ROS_INFO("Inertia : %f %f %f", vehicle_parameters_.inertia_(0,0), vehicle_parameters_.inertia_(1,1), vehicle_parameters_.inertia_(2,2));
  
  angular_acc_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;
  
  // ROS_INFO_STREAM("Gains \n" << normalized_attitude_gain_ << "\n" << normalized_angular_rate_gain_);
  // ROS_INFO_STREAM("allocation_matrix_\n" << controller_parameters_.allocation_matrix_);
  // ROS_INFO_STREAM("I \n" << I);
  ROS_INFO_STREAM("angular_acc_to_rotor_velocities_\n" << angular_acc_to_rotor_velocities_);
  // ROS_INFO_STREAM("\n"<< (controller_parameters_.allocation_matrix_
  //     * controller_parameters_.allocation_matrix_.transpose()));
  // ROS_INFO_STREAM("\n"<< controller_parameters_.allocation_matrix_.transpose()
  //     * (controller_parameters_.allocation_matrix_
  //     * controller_parameters_.allocation_matrix_.transpose()).inverse());
}

void ReconfigPidController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities, 
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

  Eigen::Vector3d acceleration;
  ComputeDesiredAcceleration(&acceleration, data_out);

  Eigen::Vector3d angular_acceleration;
  ComputeDesiredAngularAcc(acceleration, &angular_acceleration, data_out);

  // Project thrust onto body z axis.
  double thrust = -normalized_mass * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2)); //Added by Viswa

  data_out->thrust = thrust;



  const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix(); //Added by Viswa
  double velocity_W =  (R_W_I * odometry_.velocity).z(); //Added by Viswa
  
  double position_error; //Added by Viswa
  position_error = odometry_.position.z() - command_trajectory_.position_W.z(); //Added by Viswa


  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = thrust;

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
/*
  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
  I(3, 3) = 1;
  static bool control_flag=0;

  if((odometry_.velocity.z() > 0.01) || (odometry_.velocity.z() < -0.01))
  {
    control_flag = 0;
    //ROS_INFO("Control Flag is ON, %f", acceleration.z());

  }

 
  if((odometry_.velocity.z() < 0.0001) && control_flag && (odometry_.velocity.z() > -0.0001))
  {
    if ((position_error < -0.1))
    {
      double payload_mass = - (controller_parameters_.position_gain_.z() * (position_error))/vehicle_parameters_.gravity_;
      normalized_mass = vehicle_parameters_.mass_ + payload_mass;
      //UpdateMassAndInertia(new_mass);      
      ROS_INFO("%f Normalized Mass: %f", odometry_.position.z(), payload_mass);
      control_flag = 0;
      ROS_INFO("%f, %f", acceleration.z(), odometry_.velocity.z());
    }
      integral_active_ = 1;
      //ROS_INFO("Integral gain active");
  }
  
  //ROS_INFO("Feeding Calculated motor speeds");

  
  /*double test_mass = ((I.inverse() *
                  angular_acc_to_rotor_velocities_ * (*rotor_velocities))[3] +
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

/*void ReconfigPidController::UpdateMassAndInertia(double new_mass) 
{
  vehicle_parameters_.payload_mass_ = new_mass;
  normalized_mass = vehicle_parameters_.mass_ + vehicle_parameters_.payload_mass_;
}*/

void ReconfigPidController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void ReconfigPidController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void ReconfigPidController::ComputeDesiredAcceleration(Eigen::Vector3d* acceleration, 
  msg_check::PlotDataMsg* data_out) const {
  assert(acceleration);

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

  //Added by Viswa
  static Eigen::Vector3d position_error_integral;
  position_error_integral += position_error * (current_time - last_time).toSec();
  last_time = current_time;

  *acceleration = (position_error.cwiseProduct(controller_parameters_.position_gain_)
      + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)
      + position_error_integral.cwiseProduct(controller_parameters_.position_integral_gain_)) / normalized_mass
      - vehicle_parameters_.gravity_ * e_3 - command_trajectory_.acceleration_W; //Added by Viswa
  
  tf::vectorEigenToMsg(*acceleration, data_out->acceleration);
  tf::vectorEigenToMsg(position_error, data_out->position_error);
  tf::vectorEigenToMsg(velocity_error, data_out->velocity_error);
  tf::vectorEigenToMsg(position_error_integral, data_out->position_error_integral);

  //ROS_INFO("Mass : %f", normalized_mass);

  //ROS_INFO("Mass of Iris: %f, %f", vehicle_parameters_.mass_, vehicle_parameters_.payload_mass_); //Added by Viswa
}

// Implementation from the T. Pid et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void ReconfigPidController::ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                                     Eigen::Vector3d* angular_acceleration, 
                                                     msg_check::PlotDataMsg* data_out) const {
  assert(angular_acceleration);

  Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();

  // Get the desired rotation matrix.
  Eigen::Vector3d b1_des;
  double yaw = command_trajectory_.getYaw();
  b1_des << cos(yaw), sin(yaw), 0;

  Eigen::Vector3d b3_des;
  b3_des = -acceleration / acceleration.norm();

  Eigen::Vector3d b2_des;
  b2_des = b3_des.cross(b1_des);
  b2_des.normalize();

  Eigen::Matrix3d R_des;
  R_des.col(0) = b2_des.cross(b3_des);
  R_des.col(1) = b2_des;
  R_des.col(2) = b3_des;

  // Angle error according to pid et al.
  Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  Eigen::Vector3d angle_error;
  vectorFromSkewMatrix(angle_error_matrix, &angle_error);

  // TODO(burrimi) include angular rate references at some point.
  Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  angular_rate_des[2] = command_trajectory_.getYawRate();

  Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;

  

  *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
                           - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
                           + odometry_.angular_velocity.cross(odometry_.angular_velocity); // we don't need the inertia matrix here
  // data_out->angular_acceleration = *angular_acceleration;
  // data_out->angle_error = angle_error;
  // data_out->angle_rate_error = angular_rate_error;
  tf::vectorEigenToMsg(*angular_acceleration, data_out->angular_acceleration);
  tf::vectorEigenToMsg(angle_error, data_out->angle_error);
  tf::vectorEigenToMsg(angular_rate_error, data_out->angle_rate_error);
}




}
