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

#ifndef RRC_CONTROL_RSB_POSITION_CONTROLLER_H
#define RRC_CONTROL_RSB_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>


#include "rrc_control/common.h"
#include "rrc_control/parameters.h"
#include "rrc_control/PlotDataMsg.h"


namespace rrc_control {

// Default values for the sb position controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultkbp = Eigen::Vector3d(6, 6, 6);
static const Eigen::Vector3d kDefaultK1p = Eigen::Vector3d(4.7, 4.7, 4.7);
static const Eigen::Vector3d kDefaultK2p = Eigen::Vector3d(0.00, 0.00, 0.00); //Added by Viswa
static const Eigen::Vector3d kDefaultkbq = Eigen::Vector3d(0.75, 0.75, 1.57);
static const Eigen::Vector3d kDefaultK1q = Eigen::Vector3d(0.52, 0.52, 0.025);
static const Eigen::Vector3d kDefaultK2q = Eigen::Vector3d(0.52, 0.52, 0.025);
static const Eigen::Vector3d kDefaultSigma = Eigen::Vector3d(1.1, 1.1, 1.1);
static const Eigen::Vector3d kDefaultC = Eigen::Vector3d(1.1, 1.1, 1.1);
static const Eigen::Vector3d kDefaultE = Eigen::Vector3d(0.1, 0.1, 0.1);
static const Eigen::Vector3d kDefaultEeta = Eigen::Vector3d(0.001, 0.001, 0.001);

class RsbPositionControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RsbPositionControllerParameters()
      : kbp_(kDefaultkbp),
        K1p_(kDefaultK1p),
        K2p_(kDefaultK2p),
        kbq_(kDefaultkbq),
        K1q_(kDefaultK1q),
        K2q_(kDefaultK2q),
        E_hat_p_(kDefaultE),
        E_hat_q_(kDefaultE),
        sigma_p_(kDefaultSigma),
        sigma_q_(kDefaultSigma),
        c1_p_(kDefaultC),
        c1_q_(kDefaultC),
        c2_p_(kDefaultC),
        c2_q_(kDefaultC),
        eeta_p_(kDefaultEeta),
        eeta_q_(kDefaultEeta) {
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  Eigen::Matrix4Xd allocation_matrix_;
  Eigen::Vector3d kbp_;
  Eigen::Vector3d K1p_;
  Eigen::Vector3d K2p_;
  Eigen::Vector3d kbq_;
  Eigen::Vector3d K1q_;
  Eigen::Vector3d K2q_; 
  Eigen::Vector3d E_hat_p_; 
  Eigen::Vector3d E_hat_q_;
  Eigen::Vector3d sigma_p_; 
  Eigen::Vector3d sigma_q_;
  Eigen::Vector3d c1_p_; 
  Eigen::Vector3d c1_q_;
  Eigen::Vector3d c2_p_; 
  Eigen::Vector3d c2_q_;
  Eigen::Vector3d eeta_p_; 
  Eigen::Vector3d eeta_q_;
  
  RotorConfiguration rotor_configuration_;
};

class RsbPositionController {
 public:
  RsbPositionController();
  ~RsbPositionController();
  void InitializeParameters();
  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities, rrc_control::PlotDataMsg* data_out) const;

  void SetOdometry(const EigenOdometry& odometry);
  // void UpdateMassAndInertia(double new_mass); //Added by Viswa
  void SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory);

  RsbPositionControllerParameters controller_parameters_;
  VehicleParameters vehicle_parameters_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  mutable double normalized_mass; //Added by Viswa

 private:
  bool initialized_params_;
  bool controller_active_;
  mutable bool integral_active_ = 0; //Added by Viswa

  // Eigen::Vector3d normalized_attitude_gain_;
  // Eigen::Vector3d normalized_angular_rate_gain_;
  Eigen::MatrixX4d rotor_vel_coef_;

  //double test_mass; //Added by Viswa

  mav_msgs::EigenTrajectoryPoint command_trajectory_;
  EigenOdometry odometry_;
  Eigen::Vector3d J_inv;
  Eigen::Vector3d h_tilde;


  void CalculateMoments(Eigen::Vector3d force, Eigen::Vector3d* moments, rrc_control::PlotDataMsg* data_out) const;
  void CalculateThrust(Eigen::Vector3d* thrust, rrc_control::PlotDataMsg* data_out) const;
  double Sigmoid(double s, double var_pi) const;
};
}

#endif // RRC_CONTROL_RSB_POSITION_CONTROLLER_H
