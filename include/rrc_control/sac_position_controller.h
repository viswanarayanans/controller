#ifndef RRC_CONTROL_SAC_POSITION_CONTROLLER_H
#define RRC_CONTROL_SAC_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>


#include "rrc_control/common.h"
#include "rrc_control/parameters.h"
#include "msg_check/PlotDataMsg.h"


namespace rrc_control {
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


	class SacPositionControllerParameters{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		SacPositionControllerParameters()
			: theta_p_(kDefaultTheta_p),
			  theta_q_(kDefaultTheta_q),
			  alpha_p0_1_(kDefaultAlpha),
			  alpha_p1_1_(kDefaultAlpha),
			  alpha_q0_1_(kDefaultAlpha),
			  alpha_q1_1_(kDefaultAlpha),
			  alpha_q2_1_(kDefaultAlpha),
			  alpha_p0_2_(kDefaultAlpha),
			  alpha_p1_2_(kDefaultAlpha),
			  alpha_q0_2_(kDefaultAlpha),
			  alpha_q1_2_(kDefaultAlpha),
			  alpha_q2_2_(kDefaultAlpha),
			  alpha_p0_3_(kDefaultAlpha),
			  alpha_p1_3_(kDefaultAlpha),
			  alpha_q0_3_(kDefaultAlpha),
			  alpha_q1_3_(kDefaultAlpha),
			  alpha_q2_3_(kDefaultAlpha),
			  hatKp0_1_(kDefaultHatKp),
			  hatKp1_1_(kDefaultHatKp),
			  hatKq0_1_(kDefaultHatKq),
			  hatKq1_1_(kDefaultHatKq),
			  hatKq2_1_(kDefaultHatKq),
			  hatKp0_2_(kDefaultHatKp),
			  hatKp1_2_(kDefaultHatKp),
			  hatKq0_2_(kDefaultHatKq),
			  hatKq1_2_(kDefaultHatKq),
			  hatKq2_2_(kDefaultHatKq),
			  hatKp0_3_(kDefaultHatKp),
			  hatKp1_3_(kDefaultHatKp),
			  hatKq0_3_(kDefaultHatKq),
			  hatKq1_3_(kDefaultHatKq),
			  hatKq2_3_(kDefaultHatKq),
			  rho_p0_1_(kDefaultHatKp),
			  rho_p1_1_(kDefaultHatKp),
			  rho_q0_1_(kDefaultHatKq),
			  rho_q1_1_(kDefaultHatKq),
			  rho_q2_1_(kDefaultHatKq),
			  rho_p0_2_(kDefaultHatKp),
			  rho_p1_2_(kDefaultHatKp),
			  rho_q0_2_(kDefaultHatKq),
			  rho_q1_2_(kDefaultHatKq),
			  rho_q2_2_(kDefaultHatKq),
			  rho_p0_3_(kDefaultHatKp),
			  rho_p1_3_(kDefaultHatKp),
			  rho_q0_3_(kDefaultHatKq),
			  rho_q1_3_(kDefaultHatKq),
			  rho_q2_3_(kDefaultHatKq),
			  beta_p0_1_(kDefaultHatKp),
			  beta_p1_1_(kDefaultHatKp),
			  beta_q0_1_(kDefaultHatKq),
			  beta_q1_1_(kDefaultHatKq),
			  beta_q2_1_(kDefaultHatKq),
			  beta_p0_2_(kDefaultHatKp),
			  beta_p1_2_(kDefaultHatKp),
			  beta_q0_2_(kDefaultHatKq),
			  beta_q1_2_(kDefaultHatKq),
			  beta_q2_2_(kDefaultHatKq),
			  beta_p0_3_(kDefaultHatKp),
			  beta_p1_3_(kDefaultHatKp),
			  beta_q0_3_(kDefaultHatKq),
			  beta_q1_3_(kDefaultHatKq),
			  beta_q2_3_(kDefaultHatKq),
			  nu_p0_1_(kDefaultHatKp),
			  nu_p1_1_(kDefaultHatKp),
			  nu_q0_1_(kDefaultHatKq),
			  nu_q1_1_(kDefaultHatKq),
			  nu_q2_1_(kDefaultHatKq),
			  nu_p0_2_(kDefaultHatKp),
			  nu_p1_2_(kDefaultHatKp),
			  nu_q0_2_(kDefaultHatKq),
			  nu_q1_2_(kDefaultHatKq),
			  nu_q2_2_(kDefaultHatKq),
			  nu_p0_3_(kDefaultHatKp),
			  nu_p1_3_(kDefaultHatKp),
			  nu_q0_3_(kDefaultHatKq),
			  nu_q1_3_(kDefaultHatKq),
			  nu_q2_3_(kDefaultHatKq),
			  lam_p_1_(kDefaultLam_p),
			  lam_q_1_(kDefaultLam_q),
			  lam_p_2_(kDefaultLam_p),
			  lam_q_2_(kDefaultLam_q),
			  lam_p_3_(kDefaultLam_p),
			  lam_q_3_(kDefaultLam_q),
			  alpha_m_(kDefaultAlphaM),
			  hatM_1_(kDefaultHatM),
			  hatM_2_(kDefaultHatM),
			  hatM_3_(kDefaultHatM),
			  var_pi_p_(kDefaultVarPi),
			  var_pi_q_(kDefaultVarPi) {
			  	calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
			  }

		Eigen::Matrix4Xd allocation_matrix_;

		Eigen::Vector3d lam_p_1_;
		Eigen::Vector3d lam_q_1_;
		Eigen::Vector3d lam_p_2_;
		Eigen::Vector3d lam_q_2_;
		Eigen::Vector3d lam_p_3_;
		Eigen::Vector3d lam_q_3_;

		Eigen::Vector3d alpha_p0_1_;
		Eigen::Vector3d alpha_p1_1_;
		Eigen::Vector3d alpha_q0_1_;
		Eigen::Vector3d alpha_q1_1_;
		Eigen::Vector3d alpha_q2_1_;
		Eigen::Vector3d alpha_p0_2_;
		Eigen::Vector3d alpha_p1_2_;
		Eigen::Vector3d alpha_q0_2_;
		Eigen::Vector3d alpha_q1_2_;
		Eigen::Vector3d alpha_q2_2_;
		Eigen::Vector3d alpha_p0_3_;
		Eigen::Vector3d alpha_p1_3_;
		Eigen::Vector3d alpha_q0_3_;
		Eigen::Vector3d alpha_q1_3_;
		Eigen::Vector3d alpha_q2_3_;

		Eigen::Vector3d beta_p0_1_;
		Eigen::Vector3d beta_p1_1_;
		Eigen::Vector3d beta_q0_1_;
		Eigen::Vector3d beta_q1_1_;
		Eigen::Vector3d beta_q2_1_;
		Eigen::Vector3d beta_p0_2_;
		Eigen::Vector3d beta_p1_2_;
		Eigen::Vector3d beta_q0_2_;
		Eigen::Vector3d beta_q1_2_;
		Eigen::Vector3d beta_q2_2_;
		Eigen::Vector3d beta_p0_3_;
		Eigen::Vector3d beta_p1_3_;
		Eigen::Vector3d beta_q0_3_;
		Eigen::Vector3d beta_q1_3_;
		Eigen::Vector3d beta_q2_3_;

		Eigen::Vector3d nu_p0_1_;
		Eigen::Vector3d nu_p1_1_;
		Eigen::Vector3d nu_q0_1_;
		Eigen::Vector3d nu_q1_1_;
		Eigen::Vector3d nu_q2_1_;
		Eigen::Vector3d nu_p0_2_;
		Eigen::Vector3d nu_p1_2_;
		Eigen::Vector3d nu_q0_2_;
		Eigen::Vector3d nu_q1_2_;
		Eigen::Vector3d nu_q2_2_;
		Eigen::Vector3d nu_p0_3_;
		Eigen::Vector3d nu_p1_3_;
		Eigen::Vector3d nu_q0_3_;
		Eigen::Vector3d nu_q1_3_;
		Eigen::Vector3d nu_q2_3_;

		Eigen::Vector3d hatKp0_1_;
		Eigen::Vector3d hatKp1_1_;
		Eigen::Vector3d hatKq0_1_;
		Eigen::Vector3d hatKq1_1_;
		Eigen::Vector3d hatKq2_1_;
		Eigen::Vector3d hatKp0_2_;
		Eigen::Vector3d hatKp1_2_;
		Eigen::Vector3d hatKq0_2_;
		Eigen::Vector3d hatKq1_2_;
		Eigen::Vector3d hatKq2_2_;
		Eigen::Vector3d hatKp0_3_;
		Eigen::Vector3d hatKp1_3_;
		Eigen::Vector3d hatKq0_3_;
		Eigen::Vector3d hatKq1_3_;
		Eigen::Vector3d hatKq2_3_;

		Eigen::Vector3d rho_p0_1_;
		Eigen::Vector3d rho_p1_1_;
		Eigen::Vector3d rho_q0_1_;
		Eigen::Vector3d rho_q1_1_;
		Eigen::Vector3d rho_q2_1_;
		Eigen::Vector3d rho_p0_2_;
		Eigen::Vector3d rho_p1_2_;
		Eigen::Vector3d rho_q0_2_;
		Eigen::Vector3d rho_q1_2_;
		Eigen::Vector3d rho_q2_2_;
		Eigen::Vector3d rho_p0_3_;
		Eigen::Vector3d rho_p1_3_;
		Eigen::Vector3d rho_q0_3_;
		Eigen::Vector3d rho_q1_3_;
		Eigen::Vector3d rho_q2_3_;

		Eigen::Vector3d theta_p_;
		Eigen::Vector3d theta_q_;

		double alpha_m_;
		double hatM_1_;
		double hatM_2_;
		double hatM_3_;
		double var_pi_p_;
		double var_pi_q_;
		RotorConfiguration rotor_configuration_;
	};


	class SacPositionController{
	public:
		SacPositionController();
		~SacPositionController();

		void InitializeParameters();
		inline double Sigmoid(double s, double var_pi) const;
		inline double Minimum(double, double) const;
		// double Sigmoid(double s) const;
		void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities, 
                        			msg_check::PlotDataMsg* data_out);
		// void CalculateForceVector(Eigen::Vector3d* force) const;
		void CalculateThrust(Eigen::Vector3d* thrust, 
                        		msg_check::PlotDataMsg* data_out);
		void CalculateMoments(Eigen::Vector3d thrust, Eigen::Vector3d* moments, 
                        		msg_check::PlotDataMsg* data_out);
		// void CalculateThrust(Eigen::Matrix3d R_W_I, Eigen::Vector3d* thrust) const;
		// void CalculateMoments(Eigen::Vector3d force, Eigen::Vector3d* moments) const;

		void SetOdometry(const EigenOdometry& odometry);
		void SetState(const int& state);
		void SetTrajectoryPoint(
		  const mav_msgs::EigenTrajectoryPoint& command_trajectory);

		SacPositionControllerParameters controller_parameters_;
		VehicleParameters vehicle_parameters_;	

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
		bool initialized_params_;
		bool controller_active_;
		int state_;
		double varrho_p_1;
		double varrho_q_1;
		double varrho_p_2;
		double varrho_q_2;
		double varrho_p_3;
		double varrho_q_3;

		Eigen::Vector3d normalized_attitude_gain_;
		Eigen::Vector3d normalized_angular_rate_gain_;
		Eigen::MatrixX4d rotor_vel_coef_;

		mav_msgs::EigenTrajectoryPoint command_trajectory_;
		EigenOdometry odometry_;
	};
}

#endif