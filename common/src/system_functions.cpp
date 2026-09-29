#include "system_functions.hpp"

std::string mors_sys::GetEnv( const std::string & var ) 
    {
        const char * val = std::getenv( var.c_str() );
        if ( val == nullptr ) { // invalid to assign nullptr to std::string
            return "";
        }
        else {
            return val;
        }
    }

void mors_sys::quaternionToEuler(float x, float y, float z, float w, float& roll, float& pitch, float& yaw) {
    // Roll (вращение вокруг оси X)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (вращение вокруг оси Y)
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1) {
        pitch = std::copysign(M_PI / 2, sinp); // Используем ±90 градусов, если значение выходит за пределы
    } else {
        pitch = asin(sinp);
    }

    // Yaw (вращение вокруг оси Z)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);

}

MatrixXd mors_sys::euler2mat(double roll, double pitch, double yaw)
{
    double r11 = cos(yaw)*cos(pitch);
    double r12 = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll);
    double r13 = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll);
    double r21 = sin(yaw)*cos(pitch);
    double r22 = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll);
    double r23 = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);
    double r31 = -sin(pitch);
    double r32 = cos(pitch)*sin(roll);
    double r33 = cos(pitch)*cos(roll);

    // double r11 = cos(yaw);
    // double r12 = -sin(yaw);
    // double r13 = 0;
    // double r21 = sin(yaw);
    // double r22 = cos(yaw);
    // double r23 = 0;
    // double r31 = 0;
    // double r32 = 0;
    // double r33 = 1;

    MatrixXd R(3,3);
    R << r11, r12, r13,
         r21, r22, r23,
         r31, r32, r33;
    return R;
}

Matrix3d mors_sys::quat2mat(float x, float y, float z, float w)
{
    Eigen::Matrix3d R;
    R(0, 0) = -1.0 + 2.0 * (w * w) + 2.0 * (x * x);
    R(1, 1) = -1.0 + 2.0 * (w * w) + 2.0 * (y * y);
    R(2, 2) = -1.0 + 2.0 * (w * w) + 2.0 * (z * z);
    R(0, 1) = 2.0 * (x * y + w * z);
    R(0, 2) = 2.0 * (x * z - w * y);
    R(1, 0) = 2.0 * (x * y - w * z);
    R(1, 2) = 2.0 * (y * z + w * x);
    R(2, 0) = 2.0 * (x * z + w * y);
    R(2, 1) = 2.0 * (y * z - w * x);

    return R;
}

MatrixXd mors_sys::skew(VectorXd vector)
{
    MatrixXd skew_matr(3,3);
    skew_matr << 0,          -vector(2),  vector(1),
                 vector(2),   0,         -vector(0),
                -vector(1),   vector(0),  0         ;   
    return skew_matr;  

}