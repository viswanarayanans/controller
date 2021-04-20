#include <rrc_control/math.h>

Eigen::MatrixXd lyap(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Q){
	Eigen::ComplexSchur<Eigen::MatrixXd> SchurA(A);
	Eigen::MatrixXcd R = SchurA.matrixT();
	Eigen::MatrixXcd U = SchurA.matrixU();

	Eigen::ComplexSchur<Eigen::MatrixXd> SchurB(A.transpose());
	Eigen::MatrixXcd S = SchurB.matrixT();
	Eigen::MatrixXcd V = SchurB.matrixU();

	Eigen::MatrixXcd F = (U.adjoint() * Q) * V;

	Eigen::MatrixXcd Y =
	  Eigen::internal::matrix_function_solve_triangular_sylvester(R, S, F);

	Eigen::MatrixXd X = ((U * Y) * V.adjoint()).real();
	return X.real();
}