//
// Created by matthias on 11/09/18.
//

#ifndef AX18A_KINEMATICS_H
#define AX18A_KINEMATICS_H

#include <Eigen/Eigen>

class AX18AKinematics {
public:
	// DH Parameters
	const static double alpha[];
	const static double a[];
	const static double thetaOff[];
	const static double d[];

	// Create a 4x4 homogeneous transformation matrix from Denavit-Hartenberg parameters
	static Eigen::MatrixXd getTransformMatrixFromDH(double alpha, double a, double theta, double d);

	
	// Return full 4x4 homogeneous transformation matrix of end-effector frame for given joint angles
	static Eigen::MatrixXd computeForwardKinematics(Eigen::VectorXd jointAngles);

	// Return full 6x4 analytic jacobian matrix for given joint angles
	static Eigen::MatrixXd computeJacobianMatrix(Eigen::VectorXd jointAngles);

};


#endif //AX18A_KINEMATICS_H

