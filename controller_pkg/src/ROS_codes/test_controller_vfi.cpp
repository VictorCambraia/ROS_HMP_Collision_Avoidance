/**
(C) Copyright 2022 DQ Robotics Developers
This file uses the DQ Robotics library.

****************************************************
***** Coded by: Victor Cambraia N. de Oliveira *****
****************************************************

*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>

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

// global variables
std::string str_poses_human;
int refresh_pose = 0;
int stop_robot;

void cb_update_hmp(const std_msgs::String::ConstPtr& msg){
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    str_poses_human = msg->data.c_str();
    refresh_pose = 1;
    
    // std::cout << "The size is   " << str_poses_human.length()  << std::endl;
    // std::cout << str_poses_human.substr(0,100) << std::endl;
}

void cb_stop_robot(const std_msgs::Int32::ConstPtr& msg){
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    stop_robot = int(msg->data);
    // std::cout << stop_robot << std::endl;
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
    DQ new_base_robot = (franka.get_base_frame())*vi.get_object_pose("Franka")*(1+0.5*E_*(-0.07*k_));
    // DQ new_base_robot = (franka.get_base_frame())*vi.get_object_pose("Franka")*(DQ(1));
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

    // LAST TRY!!! WORKED
    DQ_QPOASESSolver solver_stop;
    DQ_ClassicQPController pose_controller(&franka, &solver_stop);

    pose_controller.set_gain(1);
    pose_controller.set_damping(1);
    pose_controller.set_control_objective(DQ_robotics::Pose);

    // controller.set_stability_threshold(0.00001);

    // VectorXd q_min = VectorXd(7);
    // q_min << -2.8973,   -1.7628,   -2.8973,   -3.0718,   -2.8973,   -0.0175,  -2.8973;

    // VectorXd q_max = VectorXd(7);
    // q_max << 2.8973,    1.7628,    2.8973,  -0.0698,    2.8973,    3.7525,    2.8973;

    // // Define some limits regarding the position and velocity of the robot joints
    // VectorXd q_minus = -(pi/2.0)*VectorXd::Ones(7);
    // VectorXd q_plus = (pi/2.0)*VectorXd::Ones(7);

    // VectorXd vel_minus = -2*VectorXd::Ones(7);
    // VectorXd vel_plus = 2*VectorXd::Ones(7);

    VectorXd q_minus(7);
    VectorXd q_plus(7);
    q_minus << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    q_plus << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;

    VectorXd vel_minus(7);
    VectorXd vel_plus(7);
    vel_minus << -2.1750, -2.1750, -2.1750, -2.1750, -2.6100, -2.6100, -2.6100;
    vel_plus << 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100;

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
    // d_safe_j << 0.05, 0.4, 0.6;
    d_safe_j << 0.5, 0.5, 0.5;

    //If you want to change the K_error factor from 0.2 to other value, write it here
    double K_error = 0.1;

    // Get the object from the class that will return the jacobian for us
    JacobianHMP J_hmp = JacobianHMP(d_safe_j, K_error);

    //Initialize the variable that will store the human poses
    int n_rows = J_hmp.num_poses*J_hmp.num_joints_per_pose;
    int n_cols = J_hmp.num_dim;
    MatrixXd poses_human = 100*MatrixXd::Ones(n_rows, n_cols);
    VectorXd deviation_joints = VectorXd::Zero(n_rows);

    // Define the pose of the camera
    // DQ pose_camera = DQ(1);
    // DQ pose_camera = 1 + 0.5*E_*(0*i_ -0.7*j_ + 0*k_);

    // Define the pose of the camera
    double ang_degree = -90;
    DQ rotation_camera = cos(ang_degree/2*(pi/180)) + sin(ang_degree/2*(pi/180))*(1*k_);
    DQ translation_camera = -0.03*i_ -0.23*j_ + 0.38*k_;
    // pose_camera_ = 1 + 0.5*E_*(0*i_ -0.7*j_ + 0*k_);
    DQ pose_camera = rotation_camera + 0.5*E_*translation_camera*rotation_camera;

    // Initialize the variables that we be used later
    double tau = 0.01; //It works as a delta_t
    VectorXd q;

    // Initialize the subscribers
    ros::NodeHandle n_pred;
    ros::Subscriber sub_prediction = n_pred.subscribe("prediction_human", 1000, cb_update_hmp);

    ros::NodeHandle n_stop;
    ros::Subscriber sub_stop_robot = n_stop.subscribe("stop_robot", 1000, cb_stop_robot);

    //   ros::spin();

    int i; //aux variable for the for
    int counter; // count the number of cycles
    int decide_td = 0; //aux variable to choose the td
    DQ td; // the desired position of the robot
    int N = 100; // Number of times that the robot will change the td

    int count_refresh = 0;
    ros::Time time_ros;
    double time_now, time_prev = 0, time_diff;

    // Define the goals of the robot (td)
    for(i=0; i<N; i++){
        if(decide_td == 0){
            td = 0.4*i_ + 0.6*j_ + 0.3*k_;
            decide_td = 2;
        }
        else if(decide_td == 1){
            td = 0.0*i_ + 0.60*j_ + 0.8*k_;
            decide_td = 2;
        }
        else if(decide_td == 2){
            td = -0.4*i_ + 0.6*j_ + 0.3*k_;
            decide_td = 0;
        }
        i = 1;
        td = 0.6*i_ + 0.1*j_ + 0.4*k_;
        
        // the translation error
        VectorXd e = VectorXd::Zero(4);
        e[0] = 1;
        // std::cout << "Starting Control Loop..." << std::endl;

        counter = 0;

        while(e.norm() > 0.05){

            // Check if it is everything okay on the ROS side, and get the callbacks
            if(ros::ok()){
                ros::spinOnce();
            }
            else{
                i = N;
                break;
            }

            if(refresh_pose == 1){
                
                if(count_refresh%60 == 0){
                    time_now = ros::Time::now().toSec();
                    time_diff = time_now - time_prev;
                    std::cout << "  \n" << time_diff << "      " << time_now << "   \n";
                    time_prev = time_now;
                }
                count_refresh++;

                // std::cout << "AQUIII 1  " << std::endl;
                // Check if this is going to work
                std::tie(poses_human, deviation_joints) = J_hmp.transform_camera_points_2matrix(str_poses_human, pose_camera);
                refresh_pose = 0;
                // std::cout << "AQUIII 2  " << std::endl;

                // TRY TO SEND THE POSE TO THE COPPELIASIM
                // Turn this into a function later
                vi.set_object_translation("Torso", DQ(poses_human.row(0)));
                vi.set_object_translation("Neck", DQ(poses_human.row(1)));
                vi.set_object_translation("Head", DQ(poses_human.row(2)));
                vi.set_object_translation("Left_Shoulder", DQ(poses_human.row(3)));
                vi.set_object_translation("Right_Shoulder", DQ(poses_human.row(4)));
                vi.set_object_translation("Left_Elbow", DQ(poses_human.row(5)));
                vi.set_object_translation("Right_Elbow", DQ(poses_human.row(6)));
                vi.set_object_translation("Left_Hand", DQ(poses_human.row(7)));
                vi.set_object_translation("Right_Hand", DQ(poses_human.row(8)));

            }


            counter = counter + 1;
            q = vi.get_joint_positions(jointnames);

            // std::cout << "AQUI 1  " << std::endl;

            MatrixXd A(0,0);
            VectorXd b(0);
            MatrixXd A_copy(0,0);
            VectorXd b_copy(0);

            // std::cout << "AQUI 2  " << std::endl;

            int n = franka.get_dim_configuration_space();
            int joint_counter; //aux variable for the for
            // Iterate it for every joint of the robot
            DQ t, x;
            for(joint_counter = n-3; joint_counter<n; joint_counter++){

                MatrixXd Jx = franka.pose_jacobian(q,joint_counter);

                // std::cout << "AQUI 4  " << std::endl;

                x = franka.fkm(q, joint_counter);
                t = translation(x);

                // Get the robot's translation Jacobian
                MatrixXd Jt = franka.translation_jacobian(Jx, x);

                // std::cout << Jt.rows() << "    " << Jt.cols() << std::endl;

                // // Get the distance Jacobian from the cube/sphere (Lets see if it works without defining the size)
                // MatrixXd Jp_p(1, n);
                // Jp_p << franka.point_to_point_distance_jacobian(Jt, t, p_sphere), MatrixXd::Zero(1, n-1-joint_counter);
                // // Jp_p << franka.point_to_point_distance_jacobian(Jt, t, p_sphere);

                // // Get the distance to the sphere 
                // double d_p_p = double(norm(t-p_sphere));
                // double d_error = pow(d_p_p,2) - pow(d_safe,2);

                // std::cout << "AQUI 5  " << std::endl;
                if((J_hmp.counter+1)%2000 == 0){
                    // std::cout << "        JOINT NUMERO      " << joint_counter << std::endl;
                }
                
                // The Jacobian for one or more poses
                MatrixXd Jp_p_aux;
                VectorXd d_error;
                // std::tie(Jp_p_aux, d_error) = J_hmp.get_jacobian_human(franka, Jt,t, points_hmp);
                // std::tie(Jp_p_aux, d_error) = J_hmp.get_jacobian_human(franka, Jt,t, poses_human, deviation_joints);
                std::tie(Jp_p_aux, d_error) = J_hmp.get_3jacobians_human(franka, Jt,t, poses_human, deviation_joints);
                MatrixXd Jp_p(Jp_p_aux.rows(),n);
                Jp_p << Jp_p_aux, MatrixXd::Zero(Jp_p_aux.rows(), n-1-joint_counter);

                // std::cout << "AQUI 5.5  " << std::endl;

                // Get the distance Jacobian to the floor
                MatrixXd Jt_pi(1,n);
                Jt_pi << franka.point_to_plane_distance_jacobian(Jt, t, pi_floor), MatrixXd::Zero(1, n-1-joint_counter);
            
                
                // Get the distance to the floor 
                double d_p_floor = double(dot(t,n_floor)-d_floor);
                double d_error_floor = d_p_floor - d_safe_floor;   

                // Define now the inequalities regarding the VFI from the cube
                MatrixXd Ap_p = -Jp_p;
                VectorXd bp_p(d_error.size());
                bp_p << nd*d_error;

                // Define now the inequalities regarding the VFI from the cube
                MatrixXd Ap_floor = -Jt_pi;
                VectorXd bp_floor(1);
                bp_floor << nd*d_error_floor;

                // std::cout << "AQUI 6  " << std::endl;

                //Define the linear inequality matrix and the linear inequality vector
                MatrixXd A_aux(Ap_p.rows() + Ap_floor.rows(), Ap_p.cols());
                A_aux << Ap_p, Ap_floor;

                // std::cout << "AQUI 7  " << std::endl;

                VectorXd b_aux(bp_p.size() + bp_floor.size());
                b_aux << bp_p, bp_floor;

                // std::cout << "AQUI 8  " << std::endl;
                A_copy.resize(A.rows(),A.cols());
                A_copy = A;

                b_copy.resize(b.size());
                b_copy = b;

                // std::cout << "AQUI 9  " << std::endl;

                // std::cout << A_copy.rows() << "    " << A_copy.cols() << std::endl;

                A.resize(A_copy.rows() + A_aux.rows(), A_aux.cols());
                b.resize(b_copy.size() + b_aux.size());

                // std::cout << "AQUI 10  " << std::endl;

                // std::cout << A.rows() << "    " << A.cols() << std::endl;
                // std::cout << A_aux.rows() << "    " << A_aux.cols() << std::endl;

                if(A_copy.size() == 0){
                    A << A_aux;
                }
                else{
                    A << A_copy, A_aux;
                }
                
                // std::cout << "AQUI 10.5  " << std::endl;

                if(b_copy.size() == 0){
                    b << b_aux;
                }
                else{
                    b << b_copy, b_aux;
                }                

                // std::cout << "AQUI 11  " << std::endl;
            }

            // Later I have to define the limits for the joints range as well
            MatrixXd W_q(14,7);
            W_q <<  -1*MatrixXd::Identity(7,7), MatrixXd::Identity(7,7);
            VectorXd w_q(14);
            w_q << -1*(q_minus - q), 1*(q_plus - q);

            // std::cout << "AQUI 12  " << std::endl;

            A_copy.resize(A.rows(),A.cols());
            A_copy = A;

            b_copy.resize(b.size());
            b_copy = b;

            A.resize(A_copy.rows() + W_q.rows(), A_copy.cols());
            b.resize(b_copy.size() + w_q.size());
            
            A << A_copy, W_q;
            b << b_copy, w_q;


            // Define the inequalities regarding the max and min velocities
            MatrixXd W_vel(14,7);
            W_vel <<  -1*MatrixXd::Identity(7,7), MatrixXd::Identity(7,7);
            VectorXd w_vel(14);
            w_vel << -1*vel_minus, 1*vel_plus;

            // std::cout << "AQUI 12  " << std::endl;

            A_copy.resize(A.rows(),A.cols());
            A_copy = A;

            b_copy.resize(b.size());
            b_copy = b;

            A.resize(A_copy.rows() + W_vel.rows(), A_copy.cols());
            b.resize(b_copy.size() + w_vel.size());
            
            A << A_copy, W_vel;
            b << b_copy, w_vel;

            // std::cout << "AQUI 13  " << std::endl;

            VectorXd u(n);
            // If there is some error/exception, mainly regarding the solver not finding a solution...
            try{
                if(stop_robot == 1){
                    // u << VectorXd::Zero(n);
                    // Probably it shouldn't be a runtime error, but okay. It is just to merge the stop_robt with the solver error 
                    throw std::runtime_error("Something is blocking the camera");
                }
                else{

                    // if(counter%4000 == 0){
                    //     ROS_INFO_STREAM(" A AMTRIX A EH " << A << " \n");
                    // }
                
                    // Update the linear inequalities in the controller
                    translation_controller.set_inequality_constraint(A, b);
                    // Get the next control signal [rad/s]
                    u << translation_controller.compute_setpoint_control_signal(q,vec4(td));  
                } 
            }
            catch(std::exception& e){
                std::cout << e.what() << std::endl;
                std::cout << "HEREEEE \n\n HEREEEEE '\n\n" << std::endl;

                MatrixXd A_stop(1,1);
                VectorXd b_stop(1);
                
                A_stop.resize(W_q.rows() + W_vel.rows(), W_q.cols());
                b_stop.resize(w_q.size() + w_vel.size());

                A_stop << W_q, W_vel;
                b_stop << w_q, w_vel;

                // Maybe add the inequality regarding the floor....
                // Update the linear inequalities in the controller
                pose_controller.set_inequality_constraint(A_stop, b_stop);
                // Get the next control signal [rad/s]
                // We put as objective the current position, so the robot try to stop
                u << pose_controller.compute_setpoint_control_signal(q,vec8(x));  
            }
            
            // Move the robot
            q = q + u*tau;
            
            // Check the error
            e = vec4(t -td);
            
            vi.set_joint_positions(jointnames, q);
        }
    }

    DQ see_error_aux = translation(franka.fkm(q));
    VectorXd see_error = vec4(see_error_aux - td);
    std::cout << see_error << std::endl;
    std::cout << "Control Finished" << std::endl;

    std::cout << "Stopping V-REP simulation..." << std::endl;
    vi.stop_simulation();
    vi.disconnect();


    return 0;
}