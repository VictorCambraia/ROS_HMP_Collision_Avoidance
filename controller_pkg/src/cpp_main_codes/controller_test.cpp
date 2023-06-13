/**
(C) Copyright 2022 DQ Robotics Developers
This file uses the DQ Robotics library.

****************************************************
***** Coded by: Victor Cambraia N. de Oliveira *****
****************************************************

*/

#include <dqrobotics/DQ.h>
#include<dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <thread>
#include <fstream>

using namespace Eigen;


int main(void)
{
    DQ_VrepInterface vi;
    vi.connect(19997,100,10);
    vi.set_synchronous(true);
    std::cout << "Starting V-REP simulation..." << std::endl;
    vi.start_simulation();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    int iterations = 10000;

    //------------------- Robot definition--------------------------
    //---------- Franka Emika Panda serial manipulator
    DQ_SerialManipulatorMDH franka = FrankaEmikaPandaRobot::kinematics();

    //Update the base of the robot from CoppeliaSim
    // DQ new_base_robot = (franka.get_base_frame())*vi.get_object_pose("Franka")*(1+0.5*E_*(-0.07*k_));
    DQ new_base_robot = (franka.get_base_frame())*vi.get_object_pose("Franka")*(DQ(1));
    franka.set_reference_frame(new_base_robot);
    //---------------------------------------------------------------

    std::vector<std::string> jointnames = {"Franka_joint1", "Franka_joint2",
                                            "Franka_joint3", "Franka_joint4",
                                            "Franka_joint5", "Franka_joint6",
                                            "Franka_joint7"};

    
    DQ_PseudoinverseController controller(&franka);
    controller.set_gain(5);
    controller.set_damping(0.05);
    controller.set_control_objective(DQ_robotics::Pose);
    // controller.set_stability_threshold(0.00001);

    // Starting here the code that I will pass
    VectorXd q_min = VectorXd(7);
    q_min << -2.8973,   -1.7628,   -2.8973,   -3.0718,   -2.8973,   -0.0175,  -2.8973;

    VectorXd q_max = VectorXd(7);
    q_max << 2.8973,    1.7628,    2.8973,  -0.0698,    2.8973,    3.7525,    2.8973;

    // Position desired
    DQ td = 0.5*i_ + 0.0*j_ + 0.5*k_;

    double ang = pi/2.0;
    DQ direction = 1*j_;
    DQ rd = cos(ang/2) + sin(ang/2)*direction;
    DQ xd = rd + 0.5*E_*td*rd;

    double tau = 0.01;
    
    VectorXd e = VectorXd::Zero(8);
    e[0] = 1;
    std::cout << "Starting Control Loop..." << std::endl;

    int counter = 0;

    // I could also check with:
    // while (not controller.system_reached_stable_region())
    while(e.norm() > 0.05){

        counter++;

        VectorXd q = vi.get_joint_positions(jointnames);
        // std::cout << "Current joint positions" << e << std::endl;
        DQ x = franka.fkm(q);
        e = vec8(x-xd);

        if(counter % 1000 == 0){
            std::cout << "Current joint positions     " << e.norm() << std::endl;
        }

        // MatrixXd J = franka.pose_jacobian(q);
        // I tried to use the pinv from the DQ library, but appeared an error
        // And using J.inverse() will throw an error for sure
        // I think I shouuld change already for the controller code
        // VectorXd u = -0.01*J.inverse()*e;
        VectorXd u = controller.compute_setpoint_control_signal(q, vec8(xd));

        q = q + u*tau;

        vi.set_joint_positions(jointnames, q);

    }

    std::cout << "Control Finished" << std::endl;

    std::cout << "Stopping V-REP simulation..." << std::endl;
    vi.stop_simulation();
    vi.disconnect();
    return 0;
}
