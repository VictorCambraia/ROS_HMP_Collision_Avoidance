#include <dqrobotics/DQ.h>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include<dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>
#include <thread>
#include <fstream>

using namespace Eigen;
using namespace DQ_robotics;

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
    DQ_SerialManipulatorMDH franka_ = FrankaEmikaPandaRobot::kinematics();

    //Update the base of the robot from CoppeliaSim
    DQ new_base_robot = (franka_.get_base_frame())*vi.get_object_pose("Franka")*(1+0.5*E_*(-0.07*k_));
    // DQ new_base_robot = (franka.get_base_frame())*vi.get_object_pose("Franka")*(DQ(1));
    franka_.set_reference_frame(new_base_robot);
    //---------------------------------------------------------------

    std::vector<std::string> jointnames = {"Franka_joint1", "Franka_joint2",
                                            "Franka_joint3", "Franka_joint4",
                                            "Franka_joint5", "Franka_joint6",
                                            "Franka_joint7"};

    DQ_QPOASESSolver solver;
    DQ_ClassicQPController translation_controller(&franka_, &solver);
    translation_controller.set_gain(5);
    translation_controller.set_damping(1);
    translation_controller.set_control_objective(DQ_robotics::Translation);
    // controller.set_stability_threshold(0.00001);

    // Starting here the code that I will pass
    VectorXd q_minus_ = VectorXd(7);
    VectorXd q_plus_ = VectorXd(7);
    q_minus_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    q_plus_ << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;

    VectorXd vel_minus_ = VectorXd(7);
    VectorXd vel_plus_ = VectorXd(7);
    vel_minus_ << -2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100;
    vel_plus_ << 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100;

    // Position desired
    DQ td_, td1_, td2_;
    td1_ =  0.4*i_ + 0.6*j_ + 0.3*k_;
    td2_ = -0.4*i_ + 0.6*j_ + 0.3*k_; 
    td_ = td1_;

    int decide_td_=0;

    double tau = 0.01;
    
    
    std::cout << "Starting Control Loop..." << std::endl;

    int counter = 0;

    // TEST ZONE
    std::array<double, 3> q_test;
    q_test[0] = 1;
    q_test[1] = 1.6;
    q_test[2] = 1.8;

    std::cout << q_test[1] << std::endl;

    VectorXd v_test(3);
    v_test[0] = q_test[0];
    v_test[1] = q_test[1];
    v_test[2] = q_test[2];

    std::cout << v_test << std::endl;


    // I could also check with:
    // while (not controller.system_reached_stable_region())
    while(1){

        
        counter++;

        VectorXd q = vi.get_joint_positions(jointnames);
        // std::cout << "Current joint positions" << e << std::endl;
        DQ x = franka_.fkm(q);
        DQ t = translation(x);

        MatrixXd Jx = franka_.pose_jacobian(q);
        // Get the robot's translation Jacobian
        MatrixXd Jt = franka_.translation_jacobian(Jx, x);

        MatrixXd W_q(14,7);
        W_q <<  -1*MatrixXd::Identity(7,7), MatrixXd::Identity(7,7);
        VectorXd w_q(14);
        w_q << -1*(q_minus_ - q), 1*(q_plus_ - q);

        // Define the inequalities regarding the max and min velocities
        MatrixXd W_vel(14,7);
        W_vel <<  -1*MatrixXd::Identity(7,7), MatrixXd::Identity(7,7);
        VectorXd w_vel(14);
        w_vel << -1*vel_minus_, 1*vel_plus_;

        MatrixXd A;
        VectorXd b;
        
        A.resize(W_q.rows() + W_vel.rows(), W_q.cols());
        b.resize(w_q.size() + w_vel.size());

        A << W_q, W_vel;
        b << w_q, w_vel;

        // Maybe add the inequality regarding the floor....
        // Update the linear inequalities in the controller
        translation_controller.set_inequality_constraint(A, b);
        // Get the next control signal [rad/s]
        // We put as objective the current position, so the robot try to stop
        VectorXd u = translation_controller.compute_setpoint_control_signal(q,vec4(td_));  

        VectorXd e = VectorXd::Zero(4);
        e = vec4(t-td_);
        if(e.norm() < 0.05){
            if(decide_td_ == 0){
                td_ = td2_;
                decide_td_ = 1;
            }
            else{
                td_ = td1_;
                decide_td_ = 0;
            }
        }

        q = q + u*tau;

        vi.set_joint_positions(jointnames, q);
    }

    std::cout << "Control Finished" << std::endl;

    std::cout << "Stopping V-REP simulation..." << std::endl;
    vi.stop_simulation();
    vi.disconnect();
    return 0;
}
