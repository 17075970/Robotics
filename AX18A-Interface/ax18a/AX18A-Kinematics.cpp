//
// Created by matthias on 11/09/18.
//


#include "AX18A-Kinematics.h"
#include <iostream>
#include <cmath>

using namespace std;
//0.055, 0.173, 0.05, 0.027
const double AX18AKinematics::alpha[] =     {M_PI / 2,  0,      M_PI / 2 + M_PI,    0};
const double AX18AKinematics::a[] =         {0.055,     0.173,  0.02,               0};
const double AX18AKinematics::thetaOff[] =  {0,         M_PI,   M_PI / 2 + M_PI,    0};
const double AX18AKinematics::d[] =         {0.095,     0,      0,                  0.192};




Eigen::MatrixXd AX18AKinematics::getTransformMatrixFromDH(double alpha, double a, double theta, double d){
    Eigen::MatrixXd m(4,4);

    m << cos(theta),    -sin(theta)*cos(alpha),     sin(theta)*sin(alpha),      a*cos(theta),
    	 sin(theta),    cos(theta)*cos(alpha),      -cos(theta)*sin(alpha),     a*sin(theta),
    	 0.0,           sin(alpha),                 cos(alpha),                 d,
    	 0.0,           0.0,                        0.0,                        1.0;

    return m;
}

// Return full 4x4 homogeneous transformation matrix of end-effector frame for given joint angles
Eigen::MatrixXd AX18AKinematics::computeForwardKinematics(Eigen::VectorXd jointAngles){
    Eigen::MatrixXd T(4,4);
    T = getTransformMatrixFromDH(alpha[0], a[0], thetaOff[0] + jointAngles[0], d[0]);
    for(int n = 1; n < 4; n++) T *= getTransformMatrixFromDH(alpha[n], a[n], thetaOff[n] + jointAngles[n], d[n]);
    return T;
}

// Return full 6x4 analytic jacobian matrix for given joint angles
Eigen::MatrixXd AX18AKinematics::computeJacobianMatrix(Eigen::VectorXd jointAngles){
    Eigen::MatrixXd fk = AX18AKinematics::computeForwardKinematics(jointAngles);



    Eigen::MatrixXd jparts[4];
    Eigen::MatrixXd J(6,4);

    Eigen::Vector3d z(0, 0, 1);
    Eigen::Vector3d o(0, 0, 0);
    Eigen::Vector3d on(fk(12), fk(13), fk(14));

    //Eigen::Vector3d p(on(0)-o(0), on(1)-o(1), on(2)-o(2));

    jparts[0] = Eigen::MatrixXd(6, 1);
    jparts[0] <<    z.cross(on - o),
                    z;

    for(int n = 1; n < 4; n++){
        Eigen::MatrixXd tm = AX18AKinematics::getTransformMatrixFromDH(alpha[0], a[0], thetaOff[0] + jointAngles[0], d[0]);
        for(int i = 1; i < n; i++) tm *= getTransformMatrixFromDH(alpha[i], a[i], thetaOff[i] + jointAngles[i], d[i]);

        Eigen::Vector3d z(tm(8), tm(9), tm(10));
        Eigen::Vector3d o(tm(12), tm(13), tm(14));
        Eigen::Vector3d on(fk(12), fk(13), fk(14));



        //Eigen::Vector3d p(on(0)-o(0), on(1)-o(1), on(2)-o(2));

        jparts[n] = Eigen::MatrixXd(6, 1);
        jparts[n] <<    z.cross(on - o),
                        z;

        //cout << jparts[n] << end << endl;
    }


    J << jparts[0], jparts[1], jparts[2], jparts[3];

    return J;
}




