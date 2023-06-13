/**
(C) Copyright 2022 DQ Robotics Developers
This file uses the DQ Robotics library.

****************************************************
***** Coded by: Victor Cambraia N. de Oliveira *****
****************************************************

*/

#include <dqrobotics/DQ.h>
#include <dqrobotics/solvers/DQ_QPOASESSolver.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <dqrobotics/robot_control/DQ_ClassicQPController.h>
#include <thread>
#include <fstream>

// I could comment these two lines, but it is good to explicity show that i am using these 2 namespaces  
using namespace Eigen;
using namespace DQ_robotics;

class JacobianHMP{

    int counter;
    double d_safe_arm;
    double d_safe_torso;
    double d_safe_head;

    public:
        JacobianHMP(const VectorXd &d_safe){
            
            counter = 0;

            if(d_safe.size() == 1){
                d_safe_arm = d_safe[0];
                d_safe_torso = d_safe[0];
                d_safe_head = d_safe[0];
            }
            else if(d_safe.size() == 3){
                d_safe_arm = d_safe[0];
                d_safe_torso = d_safe[1];
                d_safe_head = d_safe[2];
            }
        }
        // I should  actually delete it!!!
        void add_counter(int value){
            counter = counter + value;
        }

        // std::tuple<MatrixXd, VectorXd> 
        std::tuple<MatrixXd, VectorXd> get_jacobian_human(const DQ_SerialManipulatorMDH& franka, const MatrixXd &Jt,const DQ &t,const MatrixXd &points_human){

            int total_pose;
            int num_points_human = points_human.rows();
            // n_dim should be equal to 3
            int n_dim = points_human.cols();

            counter = counter + 1;

            if(num_points_human%9 != 0){
                throw std::runtime_error("The number of points for the human pose is not 9 (the expected value)");
            }
            else{
                total_pose = int(num_points_human/9);
            }

            // TODO: I NEED TO PASS ALL THE HUMAN_POINTS TO DQ FORM
            // ACTUALLY, I DONT NEED (just turn to DQ whenever necessary)
            // But I still need to think on how I am going to do that

            double d_min = 1e6;
            int i_min = -1;
            DQ point;
            int i;

            for(i=0; i <num_points_human; i++){
                
                point = DQ(points_human.row(i));
                double dist_p = double(norm(t - point));

                if(i%9 == 2){
                    dist_p = dist_p - d_safe_head;
                } 
                else if(i%9 <= 4){
                    dist_p = dist_p - d_safe_torso;
                }
                else{
                    dist_p = dist_p - d_safe_arm;
                }

                if(dist_p < d_min){
                    i_min = i;
                    d_min = dist_p;
                }
            }

            int p_min = i_min%9;
            int pose_min = int(i_min/9);

            // Now we already have the closest point
            // Next step is to check which joint is this point
            int check_plane = 0;
            int check_line = 0;
            int check_point = 0;
            double d_safe = 100;
            MatrixXd points_line, points_line1, points_line2;
            MatrixXd points_plane;

            /// Given that we are in cpp, we have from 0 to 8
            if(p_min >= 7){
                // If we got the hands, then we can have only the hand or the arm    
                check_line = 1;
                points_line.resize(2,n_dim);
                points_line << points_human.row(i_min), points_human.row(i_min-2);
                
                d_safe = d_safe_arm;
            }
            else if(p_min >= 5){
                // If we got the elbow, then we can have the elbow or the 2 arms
                check_line = 2;
                points_line1.resize(2,n_dim);
                points_line2.resize(2,n_dim);
                points_line1 << points_human.row(i_min) , points_human.row(i_min-2);
                points_line2 << points_human.row(i_min) , points_human.row(i_min+2);
                
                d_safe = d_safe_arm;
            }            
            else if(p_min >= 3){
                // If we got the sholder, then we can have the sholder, the torso or the arm
                check_plane = 1;
                points_plane.resize(3,n_dim);
                points_plane << points_human.row(i_min) , points_human.row(i_min-p_min+1) , points_human.row(i_min-p_min+2);

                check_line = 1;
                points_line.resize(2,n_dim);
                points_line << points_human.row(i_min) , points_human.row(i_min+2);

                d_safe = d_safe_torso;
            }
            else if(p_min == 2){
                // If we got the head, then we can have the head or the neck
                check_line = 1;
                points_line.resize(2,n_dim);
                points_line << points_human.row(i_min) , points_human.row(i_min-1);

                d_safe = d_safe_head;
            }
            else if(p_min == 1){
                // If we got the base of the neck, then we can have this joint or the neck
                check_plane = 1;
                points_plane.resize(3, n_dim);
                points_plane << points_human.row(i_min-1) , points_human.row(i_min+2) , points_human.row(i_min+3);

                check_line = 1;
                points_line.resize(2,n_dim);
                points_line << points_human.row(i_min) , points_human.row(i_min+1);

                d_safe = d_safe_torso;
            }
            else{
                // If we got the torso, then we can have this joint or the entire torso (plane)
                check_plane = 1;
                points_plane.resize(3, n_dim);
                points_plane << points_human.row(i_min) , points_human.row(i_min+3) , points_human.row(i_min+4);

                d_safe = d_safe_torso;
            }

            check_point = 1;
            MatrixXd point_closer = points_human.row(i_min);
            double dist_min = double(norm(t - DQ(point_closer)));   

            // For printing and debug purposes  
            if(counter%100 == 0){
                std::cout << point_closer << std::endl;
                std::cout << i_min << std::endl;
                std::cout << p_min << std::endl;
                std::cout << dist_min << std::endl;
            }

            int decide_plane;
            DQ plane;
            int decide_line, decide_line1, decide_line2;
            DQ line, line1, line2;
            double dist_t_line, dist_t_line1, dist_t_line2;

            if(check_plane == 1){
                std::tie(decide_plane, plane) = check_get_plane(points_plane, t);

                if(decide_plane == 1){

                    // Get the distance to the plane
                    DQ n_plane  = P(plane);
                    DQ d_plane = D(plane);
                    double d_p_plane = double(dot(t,n_plane)-d_plane);
                    double d_error_plane = d_p_plane - d_safe;

                    // AQUII VOLTAR AQUI DPS
                    VectorXd d_error(1);
                    d_error << d_error_plane;
                    MatrixXd jacobian = franka.point_to_plane_distance_jacobian(Jt, t, plane);
                
                    // AQUII DELETE LATER
                    if(counter%50 == 0){
                    std::cout << "PLANE" << std::endl;
                    }
                    return std::make_tuple(jacobian, d_error);     
                }
            }

            if(check_line == 1){
                std::tie(decide_line, line, dist_t_line) = check_get_line(points_line, t);
            
                if(decide_line == 1){
                    // Use the line, and to that we calculate the jacobian
                    MatrixXd jacobian = franka.point_to_line_distance_jacobian(Jt, t, line);
        
                    // Get the distance to the plane
                    DQ l_line  = P(line);
                    DQ m_line = D(line);
                    double d_p_line = double(norm(cross(t,l_line)-m_line));
        
                    double d_error_line = pow(d_p_line,2) - pow(d_safe,2);
                    VectorXd d_error(1);
                    d_error << d_error_line;

                    // AQUII DELETE LATER
                    if(counter%50 == 0){
                    std::cout << "LINE - 1 AVAILABLE" << std::endl;
                    }

                    return std::make_tuple(jacobian, d_error);
                }       
                else if(check_line == 2){
                    std::tie(decide_line1, line1, dist_t_line1) = check_get_line(points_line1, t);
                    std::tie(decide_line2, line2, dist_t_line2) = check_get_line(points_line2, t);
                
                    if(decide_line1 == 1 && decide_line2 == 0){
                        decide_line = 1;
                    }
                    else if(decide_line1 == 0 && decide_line2 == 1){
                        decide_line = 2;
                    }
                    else if(decide_line1 == 1 && decide_line2 == 1){
                        if(dist_t_line1 < dist_t_line2){
                            decide_line = 1;
                        }   
                        else{
                            decide_line = 2;
                        }
                    }
                    else{
                        decide_line = 0;
                    }

                    if(decide_line == 1){
                        line = line1;
                    }
                    else if(decide_line == 2){
                        line = line2;
                    }

                    if(decide_line != 0){
                        // Use the line, and to that we calculate the jacobian
                        MatrixXd jacobian = franka.point_to_line_distance_jacobian(Jt, t, line);
        
                        // Get the distance to the line
                        DQ l_line  = P(line);
                        DQ m_line = D(line);
                        double d_p_line = double(norm(cross(t,l_line)-m_line));
        
                        double d_error_line = pow(d_p_line,2) - pow(d_safe,2);
                        VectorXd d_error(1);
                        d_error << d_error_line;

                        // AQUII DELETE LATER
                        if(counter%50 == 0){
                        std::cout << "LINE - 2 AVAILABLE" << std::endl;
                        }

                        return std::make_tuple(jacobian, d_error);
                    }
                }

                if(check_point == 1){
                    
                    // Use only the point if the others are not correct, and to that we calculate the jacobian        
                    MatrixXd jacobian = franka.point_to_point_distance_jacobian(Jt, t, DQ(point_closer));
        
                    // Get the distance to the point
                    double d_p_p = double(norm(t-DQ(point_closer)));
                    double d_error_p = pow(d_p_p,2) - pow(d_safe,2);
        
                    VectorXd d_error(1);
                    d_error << d_error_p;

                    // AQUII DELETE LATER
                    if(counter%50 == 0){
                        std::cout << "  POINT  " << std::endl;
                    }

                    return std::make_tuple(jacobian, d_error);
                }
            }
        }

        std::tuple<int, DQ> check_get_plane(MatrixXd &points_plane, const DQ &t){

            // Get the plane first
            DQ p_plane0 = DQ(points_plane.row(0));
            DQ p_plane1 = DQ(points_plane.row(1));
            DQ p_plane2 = DQ(points_plane.row(2));
        
            DQ vet_1_0 = p_plane1 - p_plane0;
            DQ vet_2_0 = p_plane2 - p_plane0;

            DQ n_plane = cross(vet_1_0, vet_2_0);
            n_plane = (1/double(norm(n_plane)))*n_plane;

            double d_plane;
            // To guarantee that the direction of the plane is the correct one
            // Do more checks later to see if the direction is really correct
            DQ vet_t_plane = t - p_plane0;
    
            if(double(dot(vet_t_plane, n_plane)) > 0){
                d_plane = double(dot(n_plane, p_plane0));
            }
            else{
                n_plane = -1*n_plane;
                d_plane = double(dot(n_plane, p_plane0));
            }

            DQ plane = n_plane + E_*d_plane;
            double d_t_plane = double(dot(t, n_plane)) - d_plane;

            // Our point projected in the plane
            DQ t_proj = t - d_t_plane*n_plane;
        
            // Lets find out the angle of the triangle
            double ang_tri = double(dot(vet_1_0, vet_2_0));
            ang_tri = ang_tri/(double(norm(vet_1_0)*norm(vet_2_0)));
            ang_tri = acos(ang_tri)*180/pi;
        
            // Lets find out the angle of the projected point to the other points
            DQ vet_t_0 = t_proj -  p_plane0;
        
            double ang_2 = double(dot(vet_1_0, vet_t_0));
            ang_2 = ang_2/(double(norm(vet_1_0)*norm(vet_t_0)));
            ang_2 = acos(ang_2)*180/pi;
        
            double ang_3 = double(dot(vet_2_0, vet_t_0));
            ang_3 = ang_3/(double(norm(vet_2_0)*norm(vet_t_0)));
            ang_3 = acos(ang_3)*180/pi;

            int decide_r;
            if((ang_2 <= ang_tri) && (ang_3 <= ang_tri)){
                decide_r = 1;
            }     
            else{
                decide_r = 0;
            }

            return std::make_tuple(decide_r, plane);
        }

        std::tuple<int, DQ, double> check_get_line(MatrixXd &points_line, const DQ &t){

            // Get the line first
            DQ p_line0 = DQ(points_line.row(0));
            DQ p_line1 = DQ(points_line.row(1));

            DQ l_line = p_line1 - p_line0;
            l_line = (1/double(norm(l_line)))*l_line;

            DQ m_line = cross(p_line0, l_line);

            DQ line = l_line + E_*m_line;

            // See the distances of the triangle
            double d0 = double(norm(p_line0 - t));
            double d1 = double(norm(p_line1 - t));
            double l = double(norm(p_line1 - p_line0));

            int decide_r;
            double d_t_line;
            // Now check if we use the line or not
            if(pow(d0,2) > (pow(d1,2) - pow(l,2))){
                decide_r = 1;
                d_t_line = double(norm(cross(t,l_line)-m_line));
            }
            else{
                decide_r = 0;
                d_t_line = 0;
            }
            
            return std::make_tuple(decide_r, line, d_t_line);
        }

};