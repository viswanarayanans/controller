#ifndef VISWA_CONTROL_ASMC_POSITION_CONTROLLER_H
#define VISWA_CONTROL_ASMC_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>


#include "viswa_control/common.h"
#include "viswa_control/parameters.h"

namespace viswa_control {
	static const Eigen::Vector3d kDefaultTheta_p(2,2,2);
	static const Eigen::Vector3d kDefaultTheta_q(5,5,5);

	static const Eigen::Vector3d kDefaultAlpha(10,10,10);

	static const Eigen::Vector3d kDefaultHatKp(1,1,1);
	static const Eigen::Vector3d kDefaultHatKq(0.001,0.001,0.001);

	static const Eigen::Vector3d kDefaultLam_p(100, 100, 600);
	static const Eigen::Vector3d kDefaultLam_q(10, 10, 60);

	static const double kDefaultHatM = 1;
	static const double kDefaultAlphaM = 1;
	static const double kDefaultVarPi = 0.1;


	class ASmcPositionControllerParameters{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		ASmcPositionControllerParameters()
			: theta_p_(kDefaultTheta_p),
			  theta_q_(kDefaultTheta_q),
			  alpha_p_(kDefaultAlpha),
			  alpha_q0_(kDefaultAlpha),
			  alpha_q1_(kDefaultAlpha),
			  alpha_q2_(kDefaultAlpha),
			  hatKp_(kDefaultHatKp),
			  hatKq_0_(kDefaultHatKq),
			  hatKq_1_(kDefaultHatKq),
			  hatKq_2_(kDefaultHatKq),
			  lam_p_(kDefaultLam_p),
			  lam_q_(kDefaultLam_q),
			  alpha_m_(kDefaultAlphaM),
			  hatM_(kDefaultHatM),
			  var_pi_p_(kDefaultVarPi),
			  var_pi_q_(kDefaultVarPi) {
			  	calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
			  }

		Eigen::Matrix4Xd allocation_matrix_;
		Eigen::Vector3d lam_p_;
		Eigen::Vector3d lam_q_;
		Eigen::Vector3d alpha_p_;
		Eigen::Vector3d alpha_q0_;
		Eigen::Vector3d alpha_q1_;
		Eigen::Vector3d alpha_q2_;
		Eigen::Vector3d hatKp_;
		Eigen::Vector3d hatKq_0_;
		Eigen::Vector3d hatKq_1_;
		Eigen::Vector3d hatKq_2_;

		Eigen::Vector3d theta_p_;
		Eigen::Vector3d theta_q_;

		double alpha_m_;
		double hatM_;
		double var_pi_p_;
		double var_pi_q_;
		RotorConfiguration rotor_configuration_;
	};


	class ASmcPositionController{
	public:
		ASmcPositionController();
		~ASmcPositionController();

		void InitializeParameters();
		double Sigmoid(double s, double var_pi) const;
		// double Sigmoid(double s) const;
		void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;
		// void CalculateForceVector(Eigen::Vector3d* force) const;
		void CalculateThrust(Eigen::Vector3d* thrust) const;
		void CalculateMoments(Eigen::Vector3d thrust, Eigen::Vector3d* moments) const;
		// void CalculateThrust(Eigen::Matrix3d R_W_I, Eigen::Vector3d* thrust) const;
		// void CalculateMoments(Eigen::Vector3d force, Eigen::Vector3d* moments) const;

		void SetOdometry(const EigenOdometry& odometry);
		void SetTrajectoryPoint(
		  const mav_msgs::EigenTrajectoryPoint& command_trajectory);

		ASmcPositionControllerParameters controller_parameters_;
		VehicleParameters vehicle_parameters_;	

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
		bool initialized_params_;
		bool controller_active_;

		Eigen::Vector3d normalized_attitude_gain_;
		Eigen::Vector3d normalized_angular_rate_gain_;
		Eigen::MatrixX4d rotor_vel_coef_;

		mav_msgs::EigenTrajectoryPoint command_trajectory_;
		EigenOdometry odometry_;
	};
}

#endif