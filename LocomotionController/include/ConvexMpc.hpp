#ifndef _convex_mpc_hpp_
#define _convex_mpc_hpp_

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "OsqpEigen/OsqpEigen.h"
// extern "C" {
// #include "osqp.h"
// #include "workspace.h"
// }
#include <vbmath.hpp>
#include "structs.hpp"
#include "system_functions.hpp"

using namespace Eigen;
using namespace std;

typedef Eigen::SparseMatrix<double> Sparse_Matrix;

// #define NUM_LEGS 4

class ConvexMPC{
public:
    ConvexMPC();
    ~ConvexMPC();

    void set_mpc_params(double timestep, int horizon, double friction_coeff,
                        double f_min, double f_max, VectorXd &Q, VectorXd &R);
    void set_physical_params(RobotPhysicalParams &robot_params);
    VectorXd get_contact_forces(const VectorXd& x0, const VectorXd& x_ref,
                                const MatrixXd& foot_positions, const vector<int>& contact_state);

private:
    
    void calc_AB_matrices(const VectorXd& body_rpy, const MatrixXd& foot_positions,
                          MatrixXd& A_mat, MatrixXd& B_mat);
    void calc_discrete_matrices(const MatrixXd& A_mat, const MatrixXd& B_mat, const double planning_timestep,
                                MatrixXd& Ad_mat, MatrixXd& Bd_mat);
    void calc_QP_matrices(const int planning_horizon_steps, const VectorXd& x0, const VectorXd& state_ref,
                          const MatrixXd& Ad_mat, const MatrixXd& Bd_mat,
                          Sparse_Matrix& H_sparse, VectorXd& q_dense);
    void calc_constraint_matrix(const int planning_horizon_steps,
                                const Vector4d& friction_coeff, Sparse_Matrix& Ac_sparse);
    void calc_constraint_bounds(const int planning_horizon_steps, const vector<int>& contact_state,
                                     const double fz_max, const double fz_min,
                                     VectorXd& l, VectorXd& u);
    void init_hessian_pattern();
    void update_hessian_values_from_dense();

    void init_solver();
    void update_solver();

    RobotPhysicalParams robot_params;
    VectorXd cur_rpy;

    double dt;
    int horizon;
    Vector4d friction_coeff;
    double f_min;
    double f_max;
    VectorXd Qqp_diag, Rqp_diag;

    double cos_yaw;
    double sin_yaw;
    int num_legs;
    

    Matrix3d inv_inertia, inv_mass;

    Matrix3d R_z;
    MatrixXd Ac_dense;

    MatrixXd A, B;
    MatrixXd Ad, Bd;
    MatrixXd A_qp, B_qp, H_dense, weighted_B_qp;

    OsqpEigen::Solver solver;

    Sparse_Matrix H;
    VectorXd q, state_error, weighted_state_error;
    Sparse_Matrix Ac;
    VectorXd lb, ub, Xref;

    VectorXd ref_grf_yaw_aligned, ref_grf;
    double friction_l_u;
    std::vector<OSQPInt> hessian_update_indices;
};

#endif //_convex_mpc_hpp_
