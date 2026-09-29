#ifndef _system_functions_hpp_
#define _system_functions_hpp_

#include <unistd.h>
#include <iostream>
// #include <cstdlib>
#include <cmath>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace mors_sys
{
    std::string GetEnv( const std::string & var );
    void quaternionToEuler(float x, float y, float z, float w, float& roll, float& pitch, float& yaw);
    MatrixXd euler2mat(double roll, double pitch, double yaw);
    MatrixXd skew(VectorXd vector);
    Matrix3d quat2mat(float x, float y, float z, float w);
}

#endif //_system_functions_hpp_