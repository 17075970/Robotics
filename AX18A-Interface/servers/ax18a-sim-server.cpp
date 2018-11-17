
#include <iostream>
#include <boost/asio.hpp>
#include <thread>

#include "tcp/AX-18A-Comm.h"

#include "tcp/tcp_server.h"

#include "../ax18a/AX18ARobotInterface.h"

using namespace std;

/**
 * This class represents an entirely virtual AX18A robot arm.
 * It merely manages four join angles that can be read and manipulated through setting target joint angles. 
 */
class SolutionAX18ASimulatedInterface : public AX18ARobotInterface {
public:
    static std::shared_ptr<AX18ARobotInterface> create(){
        return std::shared_ptr<AX18ARobotInterface>(new SolutionAX18ASimulatedInterface());
    }

    SolutionAX18ASimulatedInterface(){
	// initialize all data members
        currentJointAngles = Eigen::VectorXd(numberOfJoints());
        currentJointAngles << 0,0,-1,0; // initial joint angles in radiant
        hasTargetJointAngles = false;
        targetJointAngles = Eigen::VectorXd();
        jointGain = 0.1;
    }

    Eigen::VectorXd getCurrentJointAngles(){
	// simply return stored value
        return currentJointAngles;
    }

    void setTargetJointAngles(Eigen::VectorXd targets){
        if(targets.rows()==0){
	    // unset target
            this->hasTargetJointAngles = false;
            return;
        }
        // check for correct dimension
        else if(targets.rows()!=numberOfJoints()){
            cerr << "Invalid number of joint angles: " << targets.rows() << endl;
        }
        else{
            this->hasTargetJointAngles = true;
            Eigen::VectorXd newTargets = targets;
            // clip angles to legal joint range
            newTargets = newTargets.cwiseMax(getJointLowerLimits());
            newTargets = newTargets.cwiseMin(getJointUpperLimits());
            this->targetJointAngles = targets;
        }
    }

    Eigen::VectorXd getTargetJointAngles(){
        if(hasTargetJointAngles) {
            return targetJointAngles;
        }
        else{
            return Eigen::VectorXd();
        }
    }

    void step(){
        if(hasTargetJointAngles){
            // move a step towards target joint anglesglPushMatrix();
            currentJointAngles += jointGain * (targetJointAngles-currentJointAngles);
        }
        // else: don't move
    }
private:
    Eigen::VectorXd currentJointAngles;
    bool hasTargetJointAngles;
    Eigen::VectorXd targetJointAngles;
    double jointGain;
};


int main(int argc, char* argv[]){
    // IMPORTANT: DO  NOT  TOUCH  THIS  METHOD
    try{
	// open network port
        boost::asio::io_service io_service;
        auto robot = SolutionAX18ASimulatedInterface::create();
        tcp_server server(io_service, AX18A_PORT, robot);
	
	// in parallel: call step() function periodically 
        thread modelUpdater([robot](){
            while(true){
                ((SolutionAX18ASimulatedInterface*)robot.get())->step();
                usleep(10*1000);
            }
        });

	// tcp_server will now delegant all network requests to methods of AX18ASimulatedInterface
        io_service.run();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
