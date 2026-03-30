#include "leg_model.hpp"

LegModel::LegModel()
{
    m1 = 0.309;
    m2 = 0.532;
    m3 = 0.049;

    l1 = 0.0595;
    l2 = 0.13;
    l3 = 0.1485;

    d1 = 0.02;
    d2 = 0.06;
    d3 = 0.012;

    l_cx_3 = 0.06;
    l_cz_2 = 0.04;

    bx = 0.1055;//0.165;
    by = 0.067;

    g = 9.81;

    J_R1.resize(3,3); J_R2.resize(3,3); J_L1.resize(3,3); J_L2.resize(3,3);
    dJ_R1.resize(3,3); dJ_R2.resize(3,3); dJ_L1.resize(3,3); dJ_L2.resize(3,3);
    R_body.resize(3,3);
}


void LegModel::set_leg_params(RobotPhysicalParams &robot)
{
    this->m1 = robot.m1;
    this->m2 = robot.m2;
    this->m3 = robot.m3;

    this->l1 = robot.l1;
    this->l2 = robot.l2;
    this->l3 = robot.l3;

    this->d1 = robot.d1;
    this->d2 = robot.d2;
    this->d3 = robot.d3;

    this->l_cx_3 = robot.l_cx_3;
    this->l_cz_2 = robot.l_cz_2;

    this->bx = robot.bx;
    this->by = robot.by;
    this->g = robot.g;
}


MatrixXd LegModel::jacobian_R1(double q1, double q2, double q3)
{
    j11 = 0;
    j12 = l2*cos(q2) + l3*cos(q2 + q3);
    j13 = l3*cos(q2 + q3);
    j21 = d1*sin(q1) + d2*sin(q1) + d3*sin(q1) + l2*cos(q1)*cos(q2) + l3*cos(q2 + q3)*cos(q1);
    j22 = -l2*sin(q1)*sin(q2) - l3*sin(q2 + q3)*sin(q1);
    j23 = -l3*sin(q2 + q3)*sin(q1);
    j31 = -d1*cos(q1) - d2*cos(q1) - d3*cos(q1) + l2*sin(q1)*cos(q2) + l3*sin(q1)*cos(q2 + q3);
    j32 = l2*sin(q2)*cos(q1) + l3*sin(q2 + q3)*cos(q1);
    j33 = l3*sin(q2 + q3)*cos(q1);

    J_R1 << j11, j12, j13,
            j21, j22, j23,
            j31, j32, j33;

    return J_R1;
}

MatrixXd LegModel::jacobian_L1(double q1, double q2, double q3)
{
    j11 = 0;
    j12 = -l2*cos(q2) - l3*cos(q2 + q3);
    j13 = -l3*cos(q2 + q3);
    j21 = -d1*sin(q1) - d2*sin(q1) - d3*sin(q1) + l2*cos(q1)*cos(q2) + l3*cos(q2 + q3)*cos(q1);
    j22 = -(l2*sin(q2) + l3*sin(q2 + q3))*sin(q1);
    j23 = -l3*sin(q2 + q3)*sin(q1);
    j31 = d1*cos(q1) + d2*cos(q1) + d3*cos(q1) + l2*sin(q1)*cos(q2) + l3*sin(q1)*cos(q2 + q3);
    j32 = l2*sin(q2)*cos(q1) + l3*sin(q2 + q3)*cos(q1);
    j33 = l3*sin(q2 + q3)*cos(q1);

    // j22 = -(l2*np.sin(q2) + l3*np.sin(q2 + q3))*np.sin(q1)
    // j23 = -l3*np.sin(q2 + q3)*np.sin(q1)
    // j31 = d1*np.cos(q1) + d2*np.cos(q1) + d3*np.cos(q1) + l2*np.sin(q1)*np.cos(q2) + l3*np.sin(q1)*np.cos(q2 + q3)
    // j32 = l2*np.sin(q2)*np.cos(q1) + l3*np.sin(q2 + q3)*np.cos(q1)
    // j33 = l3*np.sin(q2 + q3)*np.cos(q1)

    J_L1 << j11, j12, j13,
            j21, j22, j23,
            j31, j32, j33;

    return J_L1;
}

MatrixXd LegModel::jacobian_R2(double q1, double q2, double q3)
{
    j11 = 0;
    j12 = l2*cos(q2) + l3*cos(q2 + q3);
    j13 = l3*cos(q2 + q3);
    j21 = d1*sin(q1) + d2*sin(q1) + d3*sin(q1) - l2*cos(q1)*cos(q2) - l3*cos(q2 + q3)*cos(q1);
    j22 = l2*sin(q1)*sin(q2) + l3*sin(q2 + q3)*sin(q1);
    j23 = l3*sin(q2 + q3)*sin(q1);
    j31 = d1*cos(q1) + d2*cos(q1) + d3*cos(q1) + l2*sin(q1)*cos(q2) + l3*sin(q1)*cos(q2 + q3);
    j32 = l2*sin(q2)*cos(q1) + l3*sin(q2 + q3)*cos(q1);
    j33 = l3*sin(q2 + q3)*cos(q1);

    J_R2 << j11, j12, j13,
            j21, j22, j23,
            j31, j32, j33;

    return J_R2;
}

MatrixXd LegModel::jacobian_L2(double q1, double q2, double q3)
{
    j11 = 0;
    j12 = -l2*cos(q2) - l3*cos(q2 + q3);
    j13 = -l3*cos(q2 + q3);
    j21 = -d1*sin(q1) - d2*sin(q1) - d3*sin(q1) - l2*cos(q1)*cos(q2) - l3*cos(q2 + q3)*cos(q1);
    j22 = l2*sin(q1)*sin(q2) + l3*sin(q2 + q3)*sin(q1);
    j23 = l3*sin(q2 + q3)*sin(q1);
    j31 = -d1*cos(q1) - d2*cos(q1) - d3*cos(q1) + l2*sin(q1)*cos(q2) + l3*sin(q1)*cos(q2 + q3);
    j32 = l2*sin(q2)*cos(q1) + l3*sin(q2 + q3)*cos(q1);
    j33 = l3*sin(q2 + q3)*cos(q1);

    J_L2 << j11, j12, j13,
            j21, j22, j23,
            j31, j32, j33;

    return J_L2;
}

MatrixXd LegModel::d_jacobian_R1(double q1, double q2, double q3, double dq1, double dq2, double dq3)
{
    dj11 = 0;
    dj12 = -l2 * sin(q2) * dq2 - l3 * (dq2 + dq3) * sin(q2 + q3);
    dj13 = -l3 * (dq2 + dq3) * sin(q2 + q3);

    dj21 = -l2 * sin(q2) * cos(q1) * dq2 - l3 * (dq2 + dq3) * sin(q2 + q3) * cos(q1) +
           (d1 * cos(q1) + d2 * cos(q1) + d3 * cos(q1) - l2 * sin(q1) * cos(q2) - l3 * sin(q1) * cos(q2 + q3)) * dq1;
    dj22 = -(l2 * sin(q2) + l3 * sin(q2 + q3)) * cos(q1) * dq1 -
           (l2 * cos(q2) * dq2 + l3 * (dq2 + dq3) * cos(q2 + q3)) * sin(q1);
    dj23 = -l3 * (dq2 + dq3) * sin(q1) * cos(q2 + q3) - l3 * sin(q2 + q3) * cos(q1) * dq1;

    dj31 = -l2 * sin(q1) * sin(q2) * dq2 - l3 * (dq2 + dq3) * sin(q2 + q3) * sin(q1) +
           (d1 * sin(q1) + d2 * sin(q1) + d3 * sin(q1) + l2 * cos(q1) * cos(q2) + l3 * cos(q2 + q3) * cos(q1)) * dq1;
    dj32 = -(l2 * sin(q2) + l3 * sin(q2 + q3)) * sin(q1) * dq1 +
           (l2 * cos(q2) * dq2 + l3 * (dq2 + dq3) * cos(q2 + q3)) * cos(q1);
    dj33 = l3 * (dq2 + dq3) * cos(q2 + q3) * cos(q1) - l3 * sin(q2 + q3) * sin(q1) * dq1;

    dJ_R1 <<    dj11, dj12, dj13,
                dj21, dj22, dj23,
                dj31, dj32, dj33;
    return dJ_R1;
}

MatrixXd LegModel::d_jacobian_R2(double q1, double q2, double q3, double dq1, double dq2, double dq3)
{
    dj11 = 0;
    dj12 = -l2 * sin(q2) * dq2 - l3 * (dq2 + dq3) * sin(q2 + q3);
    dj13 = -l3 * (dq2 + dq3) * sin(q2 + q3);

    dj21 = l2 * sin(q2) * cos(q1) * dq2 + l3 * (dq2 + dq3) * sin(q2 + q3) * cos(q1) +
           (d1 * cos(q1) + d2 * cos(q1) + d3 * cos(q1) + l2 * sin(q1) * cos(q2) + l3 * sin(q1) * cos(q2 + q3)) * dq1;
    dj22 = (l2 * sin(q2) + l3 * sin(q2 + q3)) * cos(q1) * dq1 +
           (l2 * cos(q2) * dq2 + l3 * (dq2 + dq3) * cos(q2 + q3)) * sin(q1);
    dj23 = l3 * (dq2 + dq3) * sin(q1) * cos(q2 + q3) + l3 * sin(q2 + q3) * cos(q1) * dq1;

    dj31 = -l2 * sin(q1) * sin(q2) * dq2 - l3 * (dq2 + dq3) * sin(q2 + q3) * sin(q1) +
           (-d1 * sin(q1) - d2 * sin(q1) - d3 * sin(q1) + l2 * cos(q1) * cos(q2) + l3 * cos(q2 + q3) * cos(q1)) * dq1;
    dj32 = -(l2 * sin(q2) + l3 * sin(q2 + q3)) * sin(q1) * dq1 +
           (l2 * cos(q2) * dq2 + l3 * (dq2 + dq3) * cos(q2 + q3)) * cos(q1);
    dj33 = l3 * (dq2 + dq3) * cos(q2 + q3) * cos(q1) - l3 * sin(q2 + q3) * sin(q1) * dq1;

    dJ_R2 <<    dj11, dj12, dj13,
                dj21, dj22, dj23,
                dj31, dj32, dj33;
    return dJ_R2;
}

MatrixXd LegModel::d_jacobian_L1(double q1, double q2, double q3, double dq1, double dq2, double dq3)
{
    dj11 = 0;
    dj12 = l2 * sin(q2) * dq2 + l3 * (dq2 + dq3) * sin(q2 + q3);
    dj13 = l3 * (dq2 + dq3) * sin(q2 + q3);

    dj21 = -l2 * sin(q2) * cos(q1) * dq2 - l3 * (dq2 + dq3) * sin(q2 + q3) * cos(q1) +
           (-d1 * cos(q1) - d2 * cos(q1) - d3 * cos(q1) - l2 * sin(q1) * cos(q2) - l3 * sin(q1) * cos(q2 + q3)) * dq1;
    dj22 = -(l2 * sin(q2) + l3 * sin(q2 + q3)) * cos(q1) * dq1 -
           (l2 * cos(q2) * dq2 + l3 * (dq2 + dq3) * cos(q2 + q3)) * sin(q1);
    dj23 = -l3 * (dq2 + dq3) * sin(q1) * cos(q2 + q3) - l3 * sin(q2 + q3) * cos(q1) * dq1;

    dj31 = -l2 * sin(q1) * sin(q2) * dq2 - l3 * (dq2 + dq3) * sin(q2 + q3) * sin(q1) +
           (-d1 * sin(q1) - d2 * sin(q1) - d3 * sin(q1) + l2 * cos(q1) * cos(q2) + l3 * cos(q2 + q3) * cos(q1)) * dq1;
    dj32 = -(l2 * sin(q2) + l3 * sin(q2 + q3)) * sin(q1) * dq1 +
           (l2 * cos(q2) * dq2 + l3 * (dq2 + dq3) * cos(q2 + q3)) * cos(q1);
    dj33 = l3 * (dq2 + dq3) * cos(q2 + q3) * cos(q1) - l3 * sin(q2 + q3) * sin(q1) * dq1;

    dJ_L1 <<    dj11, dj12, dj13,
                dj21, dj22, dj23,
                dj31, dj32, dj33;
    return dJ_L1;
}

MatrixXd LegModel::d_jacobian_L2(double q1, double q2, double q3, double dq1, double dq2, double dq3)
{
    dj11 = 0;
    dj12 = l2 * sin(q2) * dq2 + l3 * (dq2 + dq3) * sin(q2 + q3);
    dj13 = l3 * (dq2 + dq3) * sin(q2 + q3);

    dj21 = l2 * sin(q2) * cos(q1) * dq2 + l3 * (dq2 + dq3) * sin(q2 + q3) * cos(q1) +
           (-d1 * cos(q1) - d2 * cos(q1) - d3 * cos(q1) + l2 * sin(q1) * cos(q2) + l3 * sin(q1) * cos(q2 + q3)) * dq1;
    dj22 = (l2 * sin(q2) + l3 * sin(q2 + q3)) * cos(q1) * dq1 +
           (l2 * cos(q2) * dq2 + l3 * (dq2 + dq3) * cos(q2 + q3)) * sin(q1);
    dj23 = l3 * (dq2 + dq3) * sin(q1) * cos(q2 + q3) + l3 * sin(q2 + q3) * cos(q1) * dq1;

    dj31 = -l2 * sin(q1) * sin(q2) * dq2 - l3 * (dq2 + dq3) * sin(q2 + q3) * sin(q1) +
           (d1 * sin(q1) + d2 * sin(q1) + d3 * sin(q1) + l2 * cos(q1) * cos(q2) + l3 * cos(q2 + q3) * cos(q1)) * dq1;
    dj32 = -(l2 * sin(q2) + l3 * sin(q2 + q3)) * sin(q1) * dq1 +
           (l2 * cos(q2) * dq2 + l3 * (dq2 + dq3) * cos(q2 + q3)) * cos(q1);
    dj33 = l3 * (dq2 + dq3) * cos(q2 + q3) * cos(q1) - l3 * sin(q2 + q3) * sin(q1) * dq1;

    dJ_L2 <<    dj11, dj12, dj13,
                dj21, dj22, dj23,
                dj31, dj32, dj33;
    return dJ_L2;
}

VectorXd LegModel::fkine_R1(double q1, double q2, double q3)
{
    double px = l1 + l2 * sin(q2) + l3 * sin(q2 + q3);
    double py = -d1 * cos(q1) - d2 * cos(q1) - d3 * cos(q1) + l2 * sin(q1) * cos(q2) + l3 * sin(q1) * cos(q2 + q3);
    double pz = -d1 * sin(q1) - d2 * sin(q1) - d3 * sin(q1) - l2 * cos(q1) * cos(q2) - l3 * cos(q2 + q3) * cos(q1);

    Eigen::VectorXd result(3); // Создаем вектор размером 3
    result << px, py, pz;
    return result;
}
VectorXd LegModel::fkine_R2(double q1, double q2, double q3)
{
    double px = -l1 + l2 * sin(q2) + l3 * sin(q2 + q3);
    double py = -d1 * cos(q1) - d2 * cos(q1) - d3 * cos(q1) - l2 * sin(q1) * cos(q2) - l3 * sin(q1) * cos(q2 + q3);
    double pz = d1 * sin(q1) + d2 * sin(q1) + d3 * sin(q1) - l2 * cos(q1) * cos(q2) - l3 * cos(q2 + q3) * cos(q1);

    Eigen::VectorXd result(3); // Создаем вектор размером 3
    result << px, py, pz;
    return result;
}
VectorXd LegModel::fkine_L1(double q1, double q2, double q3)
{
    double px = l1 - l2 * sin(q2) - l3 * sin(q2 + q3);
    double py = d1 * cos(q1) + d2 * cos(q1) + d3 * cos(q1) + l2 * sin(q1) * cos(q2) + l3 * sin(q1) * cos(q2 + q3);
    double pz = d1 * sin(q1) + d2 * sin(q1) + d3 * sin(q1) - l2 * cos(q1) * cos(q2) - l3 * cos(q2 + q3) * cos(q1);

    Eigen::VectorXd result(3); // Создаем вектор размером 3
    result << px, py, pz;
    return result;
}
VectorXd LegModel::fkine_L2(double q1, double q2, double q3)
{
    double px = -l1 - l2 * sin(q2) - l3 * sin(q2 + q3);
    double py = d1 * cos(q1) + d2 * cos(q1) + d3 * cos(q1) - l2 * sin(q1) * cos(q2) - l3 * sin(q1) * cos(q2 + q3);
    double pz = -d1 * sin(q1) - d2 * sin(q1) - d3 * sin(q1) - l2 * cos(q1) * cos(q2) - l3 * cos(q2 + q3) * cos(q1);

    Eigen::VectorXd result(3); // Создаем вектор размером 3
    result << px, py, pz;
    return result;
}

MatrixXd LegModel::body_rotation_matrix(double roll, double pitch, double yaw)
{
    r11 = cos(yaw)*cos(pitch);
    r12 = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll);
    r13 = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll);
    r21 = sin(yaw)*cos(pitch);
    r22 = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll);
    r23 = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);
    r31 = -sin(pitch);
    r32 = cos(pitch)*sin(roll);
    r33 = cos(pitch)*cos(roll);

    R_body << r11, r12, r13,
              r21, r22, r23,
              r31, r32, r33;
    return R_body;
}

void LegModel::joint_space_matrices_R1(VectorXd theta, VectorXd d_theta, MatrixXd &M, VectorXd &V, VectorXd &G, VectorXd &F)
{
    double q1 = theta(0);
    double q2 = theta(1);
    double q3 = theta(2);
    double dq1 = d_theta(0);
    double dq2 = d_theta(1);
    double dq3 = d_theta(2);

    // Вычисление часто используемых значений
    double sin_q2 = sin(q2);
    double cos_q2 = cos(q2);
    double sin_q2_plus_q3 = sin(q2 + q3);
    double cos_q2_plus_q3 = cos(q2 + q3);
    double sin_q3 = sin(q3);
    double cos_q3 = cos(q3);
    double sin_2q2 = sin(2 * q2);
    double sin_2q2_plus_q3 = sin(2 * (q2 + q3));

    // Матрица M
    double m11 = d1 * d1 * m2 + d1 * d1 * m3 + 2 * d1 * d2 * m3 + 2 * d1 * l_cz_2 * m2 + d2*d2*m3 +
                 l2 * l2 * m3 * cos_q2 * cos_q2 - 2 * l2 * l_cx_3 * m3 * sin_q2_plus_q3 * sin_q2 +
                 2 * l2 * l_cx_3 * m3 * cos_q3 - 2 * l_cx_3 * l_cx_3 * m3 * sin_q2 * sin_q3 * cos_q2_plus_q3 +
                 l_cx_3 * l_cx_3 * m3 * cos_q2 * cos_q2 + l_cx_3 * l_cx_3 * m3 * cos_q3 * cos_q3 -
                 l_cx_3 * l_cx_3 * m3 + l_cz_2 * l_cz_2 * m2;
    double m12 = -m3 * (d1 * l2 * sin_q2 + d1 * l_cx_3 * sin_q2_plus_q3 + d2 * l2 * sin_q2 + d2 * l_cx_3 * sin_q2_plus_q3);
    double m13 = -l_cx_3 * m3 * (d1 + d2) * sin_q2_plus_q3;
    double m21 = m12; // Симметричность матрицы M
    double m22 = m3 * (l2 * l2 + 2 * l2 * l_cx_3 * cos_q3 + l_cx_3 * l_cx_3);
    double m23 = l_cx_3 * m3 * (l2 * cos_q3 + l_cx_3);
    double m31 = m13; // Симметричность матрицы M
    double m32 = m23; // Симметричность матрицы M
    double m33 = l_cx_3 * l_cx_3 * m3;

    M << m11, m12, m13,
         m21, m22, m23,
         m31, m32, m33;

    // Вектор V
    double v1 = -2 * l_cx_3 * m3 * (d1 + d2) * cos_q2_plus_q3 * dq2 * dq3 -
                l_cx_3 * m3 * (d1 + d2) * cos_q2_plus_q3 * dq3 * dq3 -
                m3 * (d1 * l2 * cos_q2 + d1 * l_cx_3 * cos_q2_plus_q3 + d2 * l2 * cos_q2 + d2 * l_cx_3 * cos_q2_plus_q3) * dq2 * dq2 +
                (-l_cx_3 * m3 * (l2 * sin(2 * q2 + q3) + l2 * sin_q3 + l_cx_3 * sin_2q2_plus_q3) * dq3 -
                 m3 * (l2 * l2 * sin_2q2 / 2 + l2 * l_cx_3 * sin(2 * q2 + q3) + l_cx_3 * l_cx_3 * sin_2q2_plus_q3 / 2) * dq2) * dq1;
    double v2 = -l2 * l_cx_3 * m3 * sin_q3 * dq3 * dq3 +
                m3 * (l2 * l2 * sin_2q2 / 2 + l2 * l_cx_3 * sin(2 * q2 + q3) + l_cx_3 * l_cx_3 * sin_2q2_plus_q3 / 2) * dq1 * dq1 +
                (d2 * m3 * (l2 * cos_q2 + l_cx_3 * cos_q2_plus_q3) * dq1 - 2 * l2 * l_cx_3 * m3 * sin_q3 * dq3) * dq2;
    double v3 = d2 * l_cx_3 * m3 * cos_q2_plus_q3 * dq1 * dq2 +
                l2 * l_cx_3 * m3 * sin_q3 * dq2 * dq2 +
                l_cx_3 * m3 * (l2 * sin(2 * q2 + q3) + l2 * sin_q3 + l_cx_3 * sin_2q2_plus_q3) * dq1 * dq1 / 2;

    V << v1, v2, v3;

    // Вектор G
    double g1 = g * (d1 * m2 * sin(q1) + d1 * m3 * sin(q1) + d2 * m3 * sin(q1) +
                     l2 * m3 * cos(q1) * cos_q2 + l_cx_3 * m3 * cos_q2_plus_q3 * cos(q1) +
                     l_cz_2 * m2 * sin(q1));
    double g2 = -g * m3 * (l2 * sin_q2 + l_cx_3 * sin_q2_plus_q3) * sin(q1);
    double g3 = -g * l_cx_3 * m3 * sin_q2_plus_q3 * sin(q1);

    G << g1, g2, g3;

    // Vector F (friction)
    double k_visc = 0.2;
    double k_kul = 0.0;
    f1 = k_visc * dq1 + k_kul * vbmath::sign(dq1);
    f2 = k_visc * dq2 + k_kul * vbmath::sign(dq2);
    f3 = k_visc * dq3 + k_kul * vbmath::sign(dq3);
    F << f1, f2, f3;
}
void LegModel::joint_space_matrices_R2(VectorXd theta, VectorXd d_theta, MatrixXd &M, VectorXd &V, VectorXd &G, VectorXd &F)
{
    double q1 = theta(6);
    double q2 = theta(7);
    double q3 = theta(8);
    double dq1 = d_theta(6);
    double dq2 = d_theta(7);
    double dq3 = d_theta(8);

    // Вычисление часто используемых значений
    double sin_q2 = sin(q2);
    double cos_q2 = cos(q2);
    double sin_q2_plus_q3 = sin(q2 + q3);
    double cos_q2_plus_q3 = cos(q2 + q3);
    double sin_q3 = sin(q3);
    double cos_q3 = cos(q3);
    double sin_2q2 = sin(2 * q2);
    double sin_2q2_plus_q3 = sin(2 * (q2 + q3));

    // Матрица M
    double m11 = d1 * d1 * m2 + d1 * d1 * m3 + 2 * d1 * d2 * m3 + 2 * d1 * l_cz_2 * m2 + d2*d2*m3 + 
                 l2 * l2 * m3 * cos_q2 * cos_q2 - 2 * l2 * l_cx_3 * m3 * sin_q2_plus_q3 * sin_q2 +
                 2 * l2 * l_cx_3 * m3 * cos_q3 - 2 * l_cx_3 * l_cx_3 * m3 * sin_q2 * sin_q3 * cos_q2_plus_q3 +
                 l_cx_3 * l_cx_3 * m3 * cos_q2 * cos_q2 + l_cx_3 * l_cx_3 * m3 * cos_q3 * cos_q3 -
                 l_cx_3 * l_cx_3 * m3 + l_cz_2 * l_cz_2 * m2;
    double m12 = m3 * (d1 * l2 * sin_q2 + d1 * l_cx_3 * sin_q2_plus_q3 + d2 * l2 * sin_q2 + d2 * l_cx_3 * sin_q2_plus_q3);
    double m13 = -l_cx_3 * m3 * (d1 + d2) * sin_q2_plus_q3;
    double m21 = m12; // Симметричность матрицы M
    double m22 = m3 * (l2 * l2 + 2 * l2 * l_cx_3 * cos_q3 + l_cx_3 * l_cx_3);
    double m23 = l_cx_3 * m3 * (l2 * cos_q3 + l_cx_3);
    double m31 = -m13; // Симметричность матрицы M
    double m32 = m23;  // Симметричность матрицы M
    double m33 = l_cx_3 * l_cx_3 * m3;

    M << m11, m12, m13,
         m21, m22, m23,
         m31, m32, m33;

    // Вектор V
    double v1 = 2 * l_cx_3 * m3 * (d1 + d2) * cos_q2_plus_q3 * dq2 * dq3 +
                l_cx_3 * m3 * (d1 + d2) * cos_q2_plus_q3 * dq3 * dq3 +
                m3 * (d1 * l2 * cos_q2 + d1 * l_cx_3 * cos_q2_plus_q3 + d2 * l2 * cos_q2 + d2 * l_cx_3 * cos_q2_plus_q3) * dq2 * dq2 +
                (-l_cx_3 * m3 * (l2 * sin(2 * q2 + q3) + l2 * sin_q3 + l_cx_3 * sin_2q2_plus_q3) * dq3 -
                 m3 * (l2 * l2 * sin_2q2 / 2 + l2 * l_cx_3 * sin(2 * q2 + q3) + l_cx_3 * l_cx_3 * sin_2q2_plus_q3 / 2) * dq2) * dq1;
    double v2 = -l2 * l_cx_3 * m3 * sin_q3 * dq3 * dq3 +
                m3 * (l2 * l2 * sin_2q2 / 2 + l2 * l_cx_3 * sin(2 * q2 + q3) + l_cx_3 * l_cx_3 * sin_2q2_plus_q3 / 2) * dq1 * dq1 +
                (-d2 * m3 * (l2 * cos_q2 + l_cx_3 * cos_q2_plus_q3) * dq1 - 2 * l2 * l_cx_3 * m3 * sin_q3 * dq3) * dq2;
    double v3 = -d2 * l_cx_3 * m3 * cos_q2_plus_q3 * dq1 * dq2 +
                l2 * l_cx_3 * m3 * sin_q3 * dq2 * dq2 +
                l_cx_3 * m3 * (l2 * sin(2 * q2 + q3) + l2 * sin_q3 + l_cx_3 * sin_2q2_plus_q3) * dq1 * dq1 / 2;

    V << v1, v2, v3;

    // Вектор G
    double g1 = g * (d1 * m2 * sin(q1) + d1 * m3 * sin(q1) + d2 * m3 * sin(q1) -
                     l2 * m3 * cos(q1) * cos_q2 - l_cx_3 * m3 * cos_q2_plus_q3 * cos(q1) +
                     l_cz_2 * m2 * sin(q1));
    double g2 = g * m3 * (l2 * sin_q2 + l_cx_3 * sin_q2_plus_q3) * sin(q1);
    double g3 = g * l_cx_3 * m3 * sin_q2_plus_q3 * sin(q1);

    G << g1, g2, g3;

    // Vector F (friction)
    double k_visc = 0.2;
    double k_kul = 0.0;
    f1 = k_visc * dq1 + k_kul * vbmath::sign(dq1);
    f2 = k_visc * dq2 + k_kul * vbmath::sign(dq2);
    f3 = k_visc * dq3 + k_kul * vbmath::sign(dq3);

    F << f1, f2, f3;
}
void LegModel::joint_space_matrices_L1(VectorXd theta, VectorXd d_theta, MatrixXd &M, VectorXd &V, VectorXd &G, VectorXd &F)
{
    double q1 = theta(3);
    double q2 = theta(4);
    double q3 = theta(5);
    double dq1 = d_theta(3);
    double dq2 = d_theta(4);
    double dq3 = d_theta(5);

    // Вычисление часто используемых значений
    double sin_q2 = sin(q2);
    double cos_q2 = cos(q2);
    double sin_q2_plus_q3 = sin(q2 + q3);
    double cos_q2_plus_q3 = cos(q2 + q3);
    double sin_q3 = sin(q3);
    double cos_q3 = cos(q3);
    double sin_2q2 = sin(2 * q2);
    double sin_2q2_plus_q3 = sin(2 * (q2 + q3));

    // Матрица M
    double m11 = d1 * d1 * m2 + d1 * d1 * m3 + 2 * d1 * d2 * m3 + 2 * d1 * l_cz_2 * m2 + d2*d2*m3 + 
                 l2 * l2 * m3 * cos_q2 * cos_q2 - 2 * l2 * l_cx_3 * m3 * sin_q2_plus_q3 * sin_q2 +
                 2 * l2 * l_cx_3 * m3 * cos_q3 - 2 * l_cx_3 * l_cx_3 * m3 * sin_q2 * sin_q3 * cos_q2_plus_q3 +
                 l_cx_3 * l_cx_3 * m3 * cos_q2 * cos_q2 + l_cx_3 * l_cx_3 * m3 * cos_q3 * cos_q3 -
                 l_cx_3 * l_cx_3 * m3 + l_cz_2 * l_cz_2 * m2;
    double m12 = m3 * (d1 * l2 * sin_q2 + d1 * l_cx_3 * sin_q2_plus_q3 + d2 * l2 * sin_q2 + d2 * l_cx_3 * sin_q2_plus_q3);
    double m13 = l_cx_3 * m3 * (d1 + d2) * sin_q2_plus_q3;
    double m21 = m12; // Симметричность матрицы M
    double m22 = m3 * (l2 * l2 + 2 * l2 * l_cx_3 * cos_q3 + l_cx_3 * l_cx_3);
    double m23 = l_cx_3 * m3 * (l2 * cos_q3 + l_cx_3);
    double m31 = m13; // Симметричность матрицы M
    double m32 = m23; // Симметричность матрицы M
    double m33 = l_cx_3 * l_cx_3 * m3;

    M << m11, m12, m13,
         m21, m22, m23,
         m31, m32, m33;

    // Вектор V
    double v1 = 2 * l_cx_3 * m3 * (d1 + d2) * cos_q2_plus_q3 * dq2 * dq3 +
                l_cx_3 * m3 * (d1 + d2) * cos_q2_plus_q3 * dq3 * dq3 +
                m3 * (d1 * l2 * cos_q2 + d1 * l_cx_3 * cos_q2_plus_q3 + d2 * l2 * cos_q2 + d2 * l_cx_3 * cos_q2_plus_q3) * dq2 * dq2 +
                (-l_cx_3 * m3 * (l2 * sin(2 * q2 + q3) + l2 * sin_q3 + l_cx_3 * sin_2q2_plus_q3) * dq3 -
                 m3 * (l2 * l2 * sin_2q2 / 2 + l2 * l_cx_3 * sin(2 * q2 + q3) + l_cx_3 * l_cx_3 * sin_2q2_plus_q3 / 2) * dq2) * dq1;
    double v2 = -l2 * l_cx_3 * m3 * sin_q3 * dq3 * dq3 +
                m3 * (l2 * l2 * sin_2q2 / 2 + l2 * l_cx_3 * sin(2 * q2 + q3) + l_cx_3 * l_cx_3 * sin_2q2_plus_q3 / 2) * dq1 * dq1 +
                (-d2 * m3 * (l2 * cos_q2 + l_cx_3 * cos_q2_plus_q3) * dq1 - 2 * l2 * l_cx_3 * m3 * sin_q3 * dq3) * dq2;
    double v3 = -d2 * l_cx_3 * m3 * cos_q2_plus_q3 * dq1 * dq2 +
                l2 * l_cx_3 * m3 * sin_q3 * dq2 * dq2 +
                l_cx_3 * m3 * (l2 * sin(2 * q2 + q3) + l2 * sin_q3 + l_cx_3 * sin_2q2_plus_q3) * dq1 * dq1 / 2;

    V << v1, v2, v3;

    // Вектор G
    double g1 = g * (-d1 * m2 * sin(q1) - d1 * m3 * sin(q1) - d2 * m3 * sin(q1) +
                     l2 * m3 * cos(q1) * cos_q2 + l_cx_3 * m3 * cos_q2_plus_q3 * cos(q1) -
                     l_cz_2 * m2 * sin(q1));
    double g2 = -g * m3 * (l2 * sin_q2 + l_cx_3 * sin_q2_plus_q3) * sin(q1);
    double g3 = -g * l_cx_3 * m3 * sin_q2_plus_q3 * sin(q1);

    G << g1, g2, g3;

    // Vector F (friction)
    double k_visc = -0.2;
    double k_kul = 0.0;
    f1 = k_visc * dq1 + k_kul * vbmath::sign(dq1);
    f2 = k_visc * dq2 + k_kul * vbmath::sign(dq2);
    f3 = k_visc * dq3 + k_kul * vbmath::sign(dq3);
    F << f1, f2, f3;

}

void LegModel::joint_space_matrices_L2(VectorXd theta, VectorXd d_theta, MatrixXd &M, VectorXd &V, VectorXd &G, VectorXd &F)
{
    double q1 = theta(9);
    double q2 = theta(10);
    double q3 = theta(11);
    double dq1 = d_theta(9);
    double dq2 = d_theta(10);
    double dq3 = d_theta(11);

    // Вычисление часто используемых значений
    double sin_q2 = sin(q2);
    double cos_q2 = cos(q2);
    double sin_q2_plus_q3 = sin(q2 + q3);
    double cos_q2_plus_q3 = cos(q2 + q3);
    double sin_q3 = sin(q3);
    double cos_q3 = cos(q3);
    double sin_2q2 = sin(2 * q2);
    double sin_2q2_plus_q3 = sin(2 * (q2 + q3));

    // Матрица M
    double m11 = d1 * d1 * m2 + d1 * d1 * m3 + 2 * d1 * d2 * m3 + 2 * d1 * l_cz_2 * m2 + d2*d2*m3 + 
                 l2 * l2 * m3 * cos_q2 * cos_q2 - 2 * l2 * l_cx_3 * m3 * sin_q2_plus_q3 * sin_q2 +
                 2 * l2 * l_cx_3 * m3 * cos_q3 - 2 * l_cx_3 * l_cx_3 * m3 * sin_q2 * sin_q3 * cos_q2_plus_q3 +
                 l_cx_3 * l_cx_3 * m3 * cos_q2 * cos_q2 + l_cx_3 * l_cx_3 * m3 * cos_q3 * cos_q3 -
                 l_cx_3 * l_cx_3 * m3 + l_cz_2 * l_cz_2 * m2;
    double m12 = -m3 * (d1 * l2 * sin_q2 + d1 * l_cx_3 * sin_q2_plus_q3 + d2 * l2 * sin_q2 + d2 * l_cx_3 * sin_q2_plus_q3);
    double m13 = -l_cx_3 * m3 * (d1 + d2) * sin_q2_plus_q3;
    double m21 = m12; // Симметричность матрицы M
    double m22 = m3 * (l2 * l2 + 2 * l2 * l_cx_3 * cos_q3 + l_cx_3 * l_cx_3);
    double m23 = l_cx_3 * m3 * (l2 * cos_q3 + l_cx_3);
    double m31 = m13; // Симметричность матрицы M
    double m32 = m23; // Симметричность матрицы M
    double m33 = l_cx_3 * l_cx_3 * m3;

    M << m11, m12, m13,
         m21, m22, m23,
         m31, m32, m33;

    // Вектор V
    double v1 = -2 * l_cx_3 * m3 * (d1 + d2) * cos_q2_plus_q3 * dq2 * dq3 -
                l_cx_3 * m3 * (d1 + d2) * cos_q2_plus_q3 * dq3 * dq3 -
                m3 * (d1 * l2 * cos_q2 + d1 * l_cx_3 * cos_q2_plus_q3 + d2 * l2 * cos_q2 + d2 * l_cx_3 * cos_q2_plus_q3) * dq2 * dq2 +
                (-l_cx_3 * m3 * (l2 * sin(2 * q2 + q3) + l2 * sin_q3 + l_cx_3 * sin_2q2_plus_q3) * dq3 -
                 m3 * (l2 * l2 * sin_2q2 / 2 + l2 * l_cx_3 * sin(2 * q2 + q3) + l_cx_3 * l_cx_3 * sin_2q2_plus_q3 / 2) * dq2) * dq1;
    double v2 = -l2 * l_cx_3 * m3 * sin_q3 * dq3 * dq3 +
                m3 * (l2 * l2 * sin_2q2 / 2 + l2 * l_cx_3 * sin(2 * q2 + q3) + l_cx_3 * l_cx_3 * sin_2q2_plus_q3 / 2) * dq1 * dq1 +
                (d2 * m3 * (l2 * cos_q2 + l_cx_3 * cos_q2_plus_q3) * dq1 - 2 * l2 * l_cx_3 * m3 * sin_q3 * dq3) * dq2;
    double v3 = d2 * l_cx_3 * m3 * cos_q2_plus_q3 * dq1 * dq2 +
                l2 * l_cx_3 * m3 * sin_q3 * dq2 * dq2 +
                l_cx_3 * m3 * (l2 * sin(2 * q2 + q3) + l2 * sin_q3 + l_cx_3 * sin_2q2_plus_q3) * dq1 * dq1 / 2;

    V << v1, v2, v3;

    // Вектор G
    double g1 = -g * (d1 * m2 * sin(q1) + d1 * m3 * sin(q1) + d2 * m3 * sin(q1) +
                     l2 * m3 * cos(q1) * cos_q2 + l_cx_3 * m3 * cos_q2_plus_q3 * cos(q1) +
                     l_cz_2 * m2 * sin(q1));
    double g2 = g * m3 * (l2 * sin_q2 + l_cx_3 * sin_q2_plus_q3) * sin(q1);
    double g3 = g * l_cx_3 * m3 * sin_q2_plus_q3 * sin(q1);

    G << g1, g2, g3;

    // Vector F (friction)
    double k_visc = -0.2;
    double k_kul = 0.0;
    f1 = k_visc * dq1 + k_kul * vbmath::sign(dq1);
    f2 = k_visc * dq2 + k_kul * vbmath::sign(dq2);
    f3 = k_visc * dq3 + k_kul * vbmath::sign(dq3);

    F << f1, f2, f3;
}