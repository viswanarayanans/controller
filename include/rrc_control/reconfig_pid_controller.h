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

#ifndef RRC_CONTROL_RECONFIG_PID_CONTROLLER_H
#define RRC_CONTROL_RECONFIG_PID_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>


#include "rrc_control/common.h"
#include "rrc_control/parameters.h"
#include "msg_check/PlotDataMsg.h"
#include "msg_check/BoxMsg.h"


namespace rrc_control {

// Default values for the pid position controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);
static const Eigen::Vector3d kDefaultPositionIntegralGain = Eigen::Vector3d(0.00, 0.00, 0.00); //Added by Viswa
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 3, 0.035);
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.025);

// Default values for vehicle parameters
static const double kDefaultScissorsMass = 0.075;
static const double kDefaultBoxMass = 0.0;
static const double kDefaultRotorMass = 0.04;
static const double kDefaultRotorUnitMass = 0.3;
static const double kDefaultBatteryMass = 0.16;
static const double kDefaultRotorExtLength = 0.215;
static const double kDefaultInertiaX = 0.0347563;
static const double kDefaultInertiaY = 0.0458929;
static const double kDefaultInertiaZ = 0.0977;

static const Eigen::Vector3d kDefaultBoxDim = Eigen::Vector3d(0.2, 0.2, 0.1);
static const Eigen::Vector3d kDefaultBatteryDim = Eigen::Vector3d(0.1, 0.03, 0.03);


class ReconfigPidControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ReconfigPidControllerParameters()
      : position_gain_(kDefaultPositionGain),
        velocity_gain_(kDefaultVelocityGain),
        position_integral_gain_(kDefaultPositionIntegralGain), //Added by Viswa
        attitude_gain_(kDefaultAttitudeGain),
        angular_rate_gain_(kDefaultAngularRateGain) {
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  Eigen::Matrix4Xd allocation_matrix_;
  Eigen::Vector3d position_gain_;
  Eigen::Vector3d velocity_gain_;
  Eigen::Vector3d position_integral_gain_; //Added by Viswa
  Eigen::Vector3d attitude_gain_;
  Eigen::Vector3d angular_rate_gain_;
  RotorConfiguration rotor_configuration_;
};


class ReconfigVehicleParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ReconfigVehicleParameters()
      : scissors_mass_(kDefaultScissorsMass),
        box_mass_(kDefaultBoxMass),
        rotor_mass_(kDefaultRotorMass),
        rotor_unit_mass_(kDefaultRotorUnitMass),
        battery_mass_(kDefaultBatteryMass),
        rotor_ext_len_(kDefaultRotorExtLength),
        gravity_(9.81),

        box_dim_(kDefaultBoxDim),
        battery_dim_(kDefaultBatteryDim),

        inertia_(Eigen::Vector3d(kDefaultInertiaX, kDefaultInertiaY,
                                 kDefaultInertiaZ).asDiagonal()) {}
  double scissors_mass_;
  double box_mass_;
  double rotor_mass_;
  double rotor_unit_mass_;
  double battery_mass_;
  double rotor_ext_len_;
  const double gravity_;

  Eigen::Vector3d box_dim_;
  Eigen::Vector3d battery_dim_;
  Eigen::Matrix3d inertia_;
  RotorConfiguration rotor_configuration_;
};

class ReconfigPidController {
 public:
  ReconfigPidController();
  ~ReconfigPidController();
  void InitializeParameters();
  void ResetParameters(double box_mass, Eigen::Vector3d* box_dim);
  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities, msg_check::PlotDataMsg* data_out) const;

  void SetOdometry(const EigenOdometry& odometry);
  // void UpdateMassAndInertia(double new_mass); //Added by Viswa
  void SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory);

  ReconfigPidControllerParameters controller_parameters_;
  ReconfigVehicleParameters vehicle_parameters_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  mutable double normalized_mass; //Added by Viswa

 private:
  bool initialized_params_;
  bool controller_active_;
  mutable bool integral_active_ = 0; //Added by Viswa

  Eigen::Vector3d normalized_attitude_gain_;
  Eigen::Vector3d normalized_angular_rate_gain_;
  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;

  //double test_mass; //Added by Viswa

  mav_msgs::EigenTrajectoryPoint command_trajectory_;
  EigenOdometry odometry_;


  void ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                Eigen::Vector3d* angular_acceleration, msg_check::PlotDataMsg* data_out) const;
  void ComputeDesiredAcceleration(Eigen::Vector3d* acceleration, msg_check::PlotDataMsg* data_out) const;
};
}

#endif // RRC_CONTROL_RECONFIG_PID_CONTROLLER_H
