#ifndef RRC_CONTROL_AUTDE_POSITION_CONTROLLER_H
#define RRC_CONTROL_AUTDE_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>


#include "rrc_control/common.h"
#include "rrc_control/parameters.h"
#include "rrc_control/PlotDataMsg.h"


namespace rrc_control {
	static const Eigen::Vector3d kDefaultTheta_p(2,2,2);
	static const Eigen::Vector3d kDefaultTheta_q(5,5,5);

	static const double kDefaultAlpha = 10;

	static const double kDefaultHatKp = 1;
	static const double kDefaultHatKq = 0.001;

	static const Eigen::Vector3d kDefaultKp(4,4,4);
	static const Eigen::Vector3d kDefaultKd(2.7, 2.7, 2.7);

	static const double kDefaultHatM = 1;
	static const double kDefaultAlphaM = 1;
	static const double kDefaultVarPi = 0.1;
	static const double kDefaultGamma = 0.1;


	class AuTdePositionControllerParameters{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		AuTdePositionControllerParameters()
			: theta_p_(kDefaultTheta_p),
			  theta_q_(kDefaultTheta_q),
			  alpha_p0_(kDefaultAlpha, kDefaultAlpha, kDefaultAlpha),
			  alpha_p1_(kDefaultAlpha, kDefaultAlpha, kDefaultAlpha),
			  alpha_q0_(kDefaultAlpha, kDefaultAlpha, kDefaultAlpha),
			  alpha_q1_(kDefaultAlpha, kDefaultAlpha, kDefaultAlpha),
			  Kpp_(kDefaultKp),
			  Kdp_(kDefaultKd),
			  Kpq_(kDefaultKp),
			  Kdq_(kDefaultKd),
			  hatKp_0_(kDefaultHatKp, kDefaultHatKp, kDefaultHatKp),
			  hatKp_1_(kDefaultHatKp, kDefaultHatKp, kDefaultHatKp),
			  hatKq_0_(kDefaultHatKq, kDefaultHatKq, kDefaultHatKq),
			  hatKq_1_(kDefaultHatKq, kDefaultHatKq, kDefaultHatKq),
			  alpha_m_(kDefaultAlphaM),
			  hatM_(kDefaultHatM),
			  gamma_p_(kDefaultGamma, kDefaultGamma, kDefaultGamma),
			  gamma_q_(kDefaultGamma, kDefaultGamma, kDefaultGamma),
			  var_pi_p_(kDefaultVarPi),
			  var_pi_q_(kDefaultVarPi) {
			  	calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
			  }

		Eigen::Matrix4Xd allocation_matrix_;
		Eigen::Vector3d alpha_p0_;
		Eigen::Vector3d alpha_p1_;
		Eigen::Vector3d alpha_q0_;
		Eigen::Vector3d alpha_q1_;
		Eigen::Vector3d hatKp_0_;
		Eigen::Vector3d hatKp_1_;
		Eigen::Vector3d hatKq_0_;
		Eigen::Vector3d hatKq_1_;
		Eigen::Vector3d gamma_p_;
		Eigen::Vector3d gamma_q_;

		Eigen::Vector3d Kpp_;
		Eigen::Vector3d Kpq_;
		Eigen::Vector3d Kdp_;
		Eigen::Vector3d Kdq_;

		Eigen::Vector3d theta_p_;
		Eigen::Vector3d theta_q_;

		double alpha_m_;
		double hatM_;
		double var_pi_p_;
		double var_pi_q_;
		RotorConfiguration rotor_configuration_;
	};


	class AuTdePositionController{
	public:
		AuTdePositionController();
		~AuTdePositionController();

		void InitializeParameters();
		double Sigmoid(double s, double var_pi) const;
		void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities, 
                        			rrc_control::PlotDataMsg* data_out) const;
		void CalculateThrust(Eigen::Vector3d* thrust, 
                        		rrc_control::PlotDataMsg* data_out) const;
		void CalculateMoments(Eigen::Vector3d thrust, Eigen::Vector3d* moments, 
                        		rrc_control::PlotDataMsg* data_out) const;
		void SetOdometry(const EigenOdometry& odometry);
		void SetTrajectoryPoint(
		  const mav_msgs::EigenTrajectoryPoint& command_trajectory);

		AuTdePositionControllerParameters controller_parameters_;
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