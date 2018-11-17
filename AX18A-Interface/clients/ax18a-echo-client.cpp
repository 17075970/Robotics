
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <tcp/AX18ARemoteInterface.h>
#include <../ax18a/AX18A-Kinematics.h>

#include "tcp/AX-18A-Comm.h"

using namespace std;
using namespace Eigen;



int main(int argc, char* argv[]) {

    string remoteHost = "localhost";
    if (argc == 2) {
        remoteHost = argv[1];
    }

    shared_ptr<AX18ARobotInterface> robot = AX18ARemoteInterface::create(remoteHost);


    bool a = false;
    while (true){
        a = !a;

        VectorXd pos(4);

        if(a){
            pos << M_PI / 2, 0, 0, 0;
        }
        else{
            pos << 0, 0, 0, 0;
        }
        //pos << 0, 0, 0, 0;
        robot.get()->setTargetJointAngles(pos);

        VectorXd joints = robot->getCurrentJointAngles();



        cout << AX18AKinematics::computeJacobianMatrix(joints) << endl << endl;


        //AX18AKinematics::computeJacobianMatrix(joints);




        //VectorXd a(3);
        //a << 2, 3, 5;

        //cout << a * 2 << endl;

        usleep(1000*1000);

}
    return 0;
}