/**
(C) Copyright 2022 DQ Robotics Developers
This file uses the DQ Robotics library.

****************************************************
***** Coded by: Victor Cambraia N. de Oliveira *****
****************************************************

*/

#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_HolonomicBase.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>

namespace DQ_robotics
{

class JacobianHMP{

    double d_safe_arm;
    double d_safe_torso;
    double d_safe_head;

public:

    int counter;
    // Here the common number will be 50 poses predicted in advance
    int num_poses;
    // The n_dim is as expected 3 (x,y,z)
    int num_dim;
    // Probably I should add another variable that would represent the number of joints in each pose
    int num_joints_per_pose;

    // COnstructor
    JacobianHMP(const VectorXd &d_safe);

    // Methods
    void add_counter(int value);
    MatrixXd transform_points_human2matrix(std::string& str_numbers);
    std::tuple<MatrixXd, VectorXd> get_jacobian_human(const DQ_SerialManipulatorMDH& franka, const MatrixXd &Jt,const DQ &t,const MatrixXd &points_human);
    std::tuple<int, DQ> check_get_plane(MatrixXd &points_plane, const DQ &t);
    std::tuple<int, DQ, double> check_get_line(MatrixXd &points_line, const DQ &t);
};

}