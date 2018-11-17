
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <tcp/AX18ARemoteInterface.h>

#include "tcp/AX-18A-Comm.h"
#include <../ax18a/AX18A-Kinematics.h>

using namespace std;
using namespace Eigen;

int main(int argc, char* argv[]){

    string remoteHost = "localhost";
    if(argc==2){
        remoteHost = argv[1];
    }

    shared_ptr<AX18ARobotInterface> robot = AX18ARemoteInterface::create(remoteHost);

    //VectorXd pos(4);
    //pos << 0.875224, -0.651811, 1.76068, -2.62055;
    //robot.get()->setTargetJointAngles(pos);

    uint t = 0;
    usleep(2500*1000);
    VectorXd targetPosition(3);

    VectorXd target1(3);
    target1 << -0.2, 0, 0.2;

    VectorXd target2(3);
    target2 << 0, -0.2, 0.025;

    VectorXd target3(3);
    target3 << -0.2, 0, 0;

    targetPosition << target1;
    while(true){

        // alternate between two target coordinates
        // TODO choose better/more interesting target coordinates

        double k = 0.1;
        VectorXd currentPosition(3);

        VectorXd currentJoint = robot->getCurrentJointAngles();
        MatrixXd fk = AX18AKinematics::computeForwardKinematics(currentJoint);
        currentPosition << fk(12), fk(13), fk(14);
        cout << "Current position: " << currentPosition.transpose() << endl;

        VectorXd point(4);
        point << 1.48942, -0.641575, 0.665375, 0.563009;


        /*if(t == 300){
            targetPosition << target2;
        }
        else if(t == 500){
            targetPosition << target1;
            t = 0;
        }
        else{
            t++;
        }*/
        cout << "Target position: " << targetPosition.transpose() << endl;

	    // get current joint angles from robot

	    // TODO determine current effector position from currentJoint
        // (implement and use AX18AKinematics::computeForwardKinematics)



        MatrixXd J = AX18AKinematics::computeJacobianMatrix(currentJoint);
        J.conservativeResize(3, 4);



        VectorXd delta = k * (targetPosition - currentPosition);

        MatrixXd JJ = J * J.transpose();
        JJ = J.transpose() * JJ.inverse();

        MatrixXd I(4, 4);
        I <<    1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;



        MatrixXd move = JJ * delta + ((I-(JJ *J)) * (point - currentJoint));
        VectorXd newAngles = (VectorXd)(move + currentJoint);
        cout << JJ << endl << endl;
        // TODO write joint angles to robot
        robot.get()->setTargetJointAngles(newAngles);
        //cout << move << endl << endl << endl << endl;
        usleep(250*1000);
    }

    return 0;
}