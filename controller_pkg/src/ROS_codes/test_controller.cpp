/**
(C) Copyright 2022 DQ Robotics Developers
This file uses the DQ Robotics library.

****************************************************
***** Coded by: Victor Cambraia N. de Oliveira *****
****************************************************

*/

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "JacobianHMP.h"
#include <dqrobotics/DQ.h>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include<dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>
#include <thread>
#include <fstream>

// I could comment these two lines, but it is good to explicity show that i am using these 2 namespaces  
using namespace Eigen;
using namespace DQ_robotics;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void cb_update_hmp(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){

    // Initialize the node
    ros::init(argc, argv, "test_subscriber");

    // Initialize things related to the robot
    DQ_VrepInterface vi;
    vi.connect(19997,100,10);
    vi.set_synchronous(true);
    std::cout << "Starting V-REP simulation..." << std::endl;
    vi.start_simulation();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // int iterations = 10000;

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

    
    DQ_QPOASESSolver solver;
    DQ_ClassicQPController translation_controller(&franka, &solver);

    translation_controller.set_gain(5);
    translation_controller.set_damping(0.05);
    translation_controller.set_control_objective(DQ_robotics::Translation);
    // controller.set_stability_threshold(0.00001);

    // VectorXd q_min = VectorXd(7);
    // q_min << -2.8973,   -1.7628,   -2.8973,   -3.0718,   -2.8973,   -0.0175,  -2.8973;

    // VectorXd q_max = VectorXd(7);
    // q_max << 2.8973,    1.7628,    2.8973,  -0.0698,    2.8973,    3.7525,    2.8973;

    // Define some limits regarding the position and velocity of the robot joints
    VectorXd q_minus = -(pi/2.0)*VectorXd::Ones(7);
    VectorXd q_plus = (pi/2.0)*VectorXd::Ones(7);

    VectorXd vel_minus = -2*VectorXd::Ones(7);
    VectorXd vel_plus = 2*VectorXd::Ones(7);

    // ------------------- NOT USED NOW --------------------//

    // // Define the position and size of the cube/sphere
    // double size_cube = 0.2;
    // DQ p_sphere = 0.5*j_ + 0.1*k_;
    // double radius = sqrt(3)/2*size_cube;

    // // Define the velocity of the cube (Not used right now)
    // // DQ vel_cube = 1.0*k_;

    // // Define the safe distance from the object:
    // double d_safe = radius + 0.05;

    // ------------------------------------------------------//

    // Pose Human 1 (Franka_human1)
    MatrixXd points_human(9,3);
    points_human << 0.0,1,0.5, 0.0,1,0.8, 0.0,1,1.0, 0.3,1,0.8, -0.3,1,0.8, 0.3,1,0.5, -0.3,1,0.5, 0.3,0.7,0.5, -0.3,0.7,0.5;

    MatrixXd points_hmp = points_human;

    std::cout << "Points_hmp  rows  " << points_hmp.rows() << "   cols   "<< points_hmp.cols() << std::endl;

    // Define the floor
    DQ n_floor = 1*k_;
    // d_floor = DQ(-0.2);
    DQ d_floor = DQ(0);
    DQ pi_floor = n_floor + E_*d_floor;

    // Define parameters for the VFI
    // nd = 1;
    double nd = 0.4;
    double d_safe_floor = 0.1;

    // Define the safety parameters for the human
    VectorXd d_safe_j(3);
    d_safe_j << 0.02, 0.2, 0.4;

    // Get the object from the class that will return the jacobian for us
    JacobianHMP J_hmp = JacobianHMP(d_safe_j);

    // Initialize the variables that we be used later
    double tau = 0.01; //It works as a delta_t
    VectorXd q;

    ros::NodeHandle n;
    ros::Subscriber sub_prediction = n.subscribe("prediction_human", 1000, cb_update_hmp);

    //   ros::spin();

    while(ros::ok()){




        ros::spinOnce();
    }

  return 0;
}