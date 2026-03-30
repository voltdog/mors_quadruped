#ifndef _leg_model_hpp_
#define _leg_model_hpp_

#include <iostream>
#include <Eigen/Dense>
#include <vbmath.hpp>
#include "structs.hpp"

using namespace Eigen;
using namespace std;

class LegModel{
public:
    LegModel();

    void set_leg_params(RobotPhysicalParams &robot);

    MatrixXd jacobian_R1(double q1, double q2, double q3);
    MatrixXd jacobian_R2(double q1, double q2, double q3);
    MatrixXd jacobian_L1(double q1, double q2, double q3);
    MatrixXd jacobian_L2(double q1, double q2, double q3);

    MatrixXd d_jacobian_R1(double q1, double q2, double q3, double dq1, double dq2, double dq3);
    MatrixXd d_jacobian_R2(double q1, double q2, double q3, double dq1, double dq2, double dq3);
    MatrixXd d_jacobian_L1(double q1, double q2, double q3, double dq1, double dq2, double dq3);
    MatrixXd d_jacobian_L2(double q1, double q2, double q3, double dq1, double dq2, double dq3);

    VectorXd fkine_R1(double q1, double q2, double q3);
    VectorXd fkine_R2(double q1, double q2, double q3);
    VectorXd fkine_L1(double q1, double q2, double q3);
    VectorXd fkine_L2(double q1, double q2, double q3);
    MatrixXd body_rotation_matrix(double roll, double pitch, double yaw);

    void joint_space_matrices_R1(VectorXd theta, VectorXd d_theta, MatrixXd &M, VectorXd &V, VectorXd &G, VectorXd &F);
    void joint_space_matrices_R2(VectorXd theta, VectorXd d_theta, MatrixXd &M, VectorXd &V, VectorXd &G, VectorXd &F);
    void joint_space_matrices_L1(VectorXd theta, VectorXd d_theta, MatrixXd &M, VectorXd &V, VectorXd &G, VectorXd &F);
    void joint_space_matrices_L2(VectorXd theta, VectorXd d_theta, MatrixXd &M, VectorXd &V, VectorXd &G, VectorXd &F);

private:
    double m1, m2, m3;
    double l1, l2, l3;
    double d1, d2, d3;
    double l_cx_3, l_cz_2;
    double bx, by;
    double g;

    double j11, j12, j13, j21, j22, j23, j31, j32, j33;
    double dj11, dj12, dj13, dj21, dj22, dj23, dj31, dj32, dj33;
    double r11, r12, r13, r21, r22, r23, r31, r32, r33;

    double f1, f2, f3;

    MatrixXd J_R1, J_R2, J_L1, J_L2;
    MatrixXd dJ_R1, dJ_R2, dJ_L1, dJ_L2;
    MatrixXd R_body;

};

#endif //_leg_model_hpp_