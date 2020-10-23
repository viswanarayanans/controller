#ifndef RRC_CONTROL_ELAS_SMC_POSITION_CONTROLLER_H
#define RRC_CONTROL_ELAS_SMC_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>


#include "rrc_control/common.h"
#include "rrc_control/parameters.h"
#include "msg_check/PlotDataMsg.h"


namespace rrc_control {
	static const Eigen::Vector3d kDefaultTheta_p(2, 2, 2);
	static const Eigen::Vector3d kDefaultTheta_q(5, 5, 5);


	static const Eigen::Vector3d kDefaultFq(1, 1, 1);
	static const Eigen::Vector3d kDefaultInertia(0.01, 0.01, 0.02);

	static const double kDefaultEeta = 0.2;
	static const double kDefaultFp = 0.25;
	static const double kDefaultPayload = 0.5;
	static const double kDefaultRotorM = 0.05;
	static const double kDefaultVarPi = 0.1;


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



	class ElasSmcPositionControllerParameters{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		ElasSmcPositionControllerParameters()
			: theta_p_(kDefaultTheta_p),
			  theta_q_(kDefaultTheta_q),
			  eeta_p_(kDefaultEeta*Eigen::Vector3d(1,1,1)),
			  eeta_q_(kDefaultEeta*Eigen::Vector3d(1,1,1)),
			  var_pi_p_(kDefaultVarPi),
			  var_pi_q_(kDefaultVarPi) {
			  calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
			  }

		Eigen::Matrix4Xd allocation_matrix_;		

		Eigen::Vector3d theta_p_;
		Eigen::Vector3d theta_q_;

		Eigen::Vector3d eeta_p_;
		Eigen::Vector3d eeta_q_;
		double var_pi_p_;
		double var_pi_q_;
		RotorConfiguration rotor_configuration_;
	};

	class ElasSmcVehicleParameters {
	 public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	  ElasSmcVehicleParameters()
	      : scissors_mass_(kDefaultScissorsMass),
	        box_mass_(kDefaultBoxMass),
	        rotor_mass_(kDefaultRotorMass),
	        rotor_unit_mass_(kDefaultRotorUnitMass),
	        battery_mass_(kDefaultBatteryMass),
	        rotor_ext_len_(kDefaultRotorExtLength),
	        gravity_(9.81),

	        box_dim_(kDefaultBoxDim),
	        box_hat_(kDefaultBoxDim),
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
	  Eigen::Vector3d box_hat_;
	  Eigen::Vector3d battery_dim_;
	  Eigen::Matrix3d inertia_;
	  RotorConfiguration rotor_configuration_;
	};



	class ElasSmcPositionController{
	public:
		ElasSmcPositionController();
		~ElasSmcPositionController();

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

		ElasSmcPositionControllerParameters controller_parameters_;
		ElasSmcVehicleParameters vehicle_parameters_;


		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	private:
		bool initialized_params_;
		bool controller_active_;
		
		double Fp;
		double m_hat;
		Eigen::Vector3d J_hat;
		Eigen::Vector3d C_hat;
		Eigen::Vector3d Fq1;
		Eigen::Vector3d Fq2;

		Eigen::Vector3d normalized_attitude_gain_;
		Eigen::Vector3d normalized_angular_rate_gain_;
		Eigen::MatrixX4d rotor_vel_coef_;

		mav_msgs::EigenTrajectoryPoint command_trajectory_;
		EigenOdometry odometry_;
	};
}

#endif