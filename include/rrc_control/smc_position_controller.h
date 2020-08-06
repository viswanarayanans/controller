#ifndef RRC_CONTROL_SMC_POSITION_CONTROLLER_H
#define RRC_CONTROL_SMC_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>


#include "rrc_control/common.h"
#include "rrc_control/parameters.h"
#include "msg_check/PlotDataMsg.h"


namespace rrc_control {
	static const Eigen::Vector3d kDefaultTheta_p(2, 2, 2);
	static const Eigen::Vector3d kDefaultTheta_q(5, 5, 5);


	static const Eigen::Vector3d kDefaultFq(1, 1, 1);
	static const Eigen::Vector3d kDefaultJCap(0.01, 0.01, 0.02);
	static const Eigen::Vector3d kDefaultCCap(0, 0, 0.02);
	static const Eigen::Vector3d kDefaultInertia(0.01, 0.01, 0.02);


	static const double kDefaultMCap = 2;
	static const double kDefaultEeta = 0.2;
	static const double kDefaultFp = 0.25;
	static const double kDefaultPayload = 0.5;
	static const double kDefaultRotorM = 0.05;
	static const double kDefaultVarPi = 0.1;
	static const double kDefaultGra = 9.8;
	static const double kDefaultLength = 0.2;


	class SmcPositionControllerParameters{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		SmcPositionControllerParameters()
			: theta_p_(kDefaultTheta_p),
			  theta_q_(kDefaultTheta_q),
			  eeta_p_(kDefaultEeta*Eigen::Vector3d(1,1,1)),
			  eeta_q_(kDefaultEeta),
			  Fq1_(kDefaultFq),
			  Fq2_(kDefaultFq),
			  Fp_(kDefaultFp),
			  J_cap_(kDefaultJCap),
			  C_cap_(kDefaultCCap),
			  m_cap_(kDefaultMCap),
			  gravity_(kDefaultGra),
			  payload_(kDefaultPayload),
			  rotor_mass_(kDefaultRotorM),
			  length_(kDefaultLength),
			  arm_length_(kDefaultLength),
			  side_(kDefaultLength),
			  var_pi_p_(kDefaultVarPi),
			  var_pi_q_(kDefaultVarPi),
			  inertia_(kDefaultInertia.asDiagonal()) {
			  calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
			  }

		Eigen::Matrix4Xd allocation_matrix_;
		Eigen::Matrix3d inertia_;

		Eigen::Vector3d J_cap_;
		Eigen::Vector3d C_cap_;
		Eigen::Vector3d Fq1_;
		Eigen::Vector3d Fq2_;
		

		Eigen::Vector3d theta_p_;
		Eigen::Vector3d theta_q_;

		double m_cap_;
		double payload_;
		double rotor_mass_;
		double length_;
		double arm_length_;
		double side_;
		double Fp_;
		double gravity_;

		Eigen::Vector3d eeta_p_;
		double eeta_q_;
		double var_pi_p_;
		double var_pi_q_;
		RotorConfiguration rotor_configuration_;
	};


	class SmcPositionController{
	public:
		SmcPositionController();
		~SmcPositionController();

		void InitializeParameters();
		double Sigmoid(double s, double var_pi) const;
		// double Sigmoid(double s) const;
		void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities, 
                                     msg_check::PlotDataMsg* data_out) const;
		// void CalculateForceVector(Eigen::Vector3d* force) const;
		void ComputeThrust(Eigen::Vector3d* thrust, 
                                     msg_check::PlotDataMsg* data_out) const;
		void CalculateMoments(Eigen::Vector3d force, Eigen::Vector3d* moments, 
                                     msg_check::PlotDataMsg* data_out) const;
		// void CalculateThrust(Eigen::Matrix3d R_W_I, Eigen::Vector3d* thrust) const;
		// void CalculateMoments(Eigen::Vector3d force, Eigen::Vector3d* moments) const;

		void SetOdometry(const EigenOdometry& odometry);
		void SetTrajectoryPoint(
		  const mav_msgs::EigenTrajectoryPoint& command_trajectory);

		SmcPositionControllerParameters controller_parameters_;
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