#include "ConvexMpc.hpp"
#include <vector>

namespace {
bool update_solver_data(OsqpEigen::Solver& solver,
                        const Sparse_Matrix& hessian,
                        const std::vector<OSQPInt>& hessian_update_indices,
                        const VectorXd& gradient,
                        const VectorXd& lower_bound,
                        const VectorXd& upper_bound)
{
#ifdef OSQP_EIGEN_OSQP_IS_V1
    const auto& osqp_solver = solver.solver();
    if (!osqp_solver) {
        return false;
    }

    if (osqp_update_data_mat(osqp_solver.get(),
                             hessian.valuePtr(),
                             hessian_update_indices.data(),
                             static_cast<OSQPInt>(hessian_update_indices.size()),
                             OSQP_NULL,
                             OSQP_NULL,
                             0)
        != 0) {
        return false;
    }

    return osqp_update_data_vec(osqp_solver.get(),
                                gradient.data(),
                                lower_bound.data(),
                                upper_bound.data())
        == 0;
#else
    return solver.updateHessianMatrix(hessian)
        && solver.updateGradient(gradient)
        && solver.updateBounds(lower_bound, upper_bound);
#endif
}
}

// #include "osqp.h"
// #include "workspace.h"


ConvexMPC::ConvexMPC()
{
    num_legs = NUM_LEGS;
    cur_rpy.resize(3);
    A.resize(13, 13);
    B.resize(13, 12);
    Ad.resize(13, 13);
    Bd.resize(13, 12);
    ref_grf.resize(12);
    ref_grf_yaw_aligned.resize(12);
}

ConvexMPC::~ConvexMPC()
{
    
}

void ConvexMPC::set_physical_params(RobotPhysicalParams &robot_params)
{
    this->robot_params = robot_params;
    inv_mass = Eigen::Matrix3d::Identity()/robot_params.M_b;
    
}

void ConvexMPC::set_mpc_params(double timestep, int horizon, double friction_coeff,
    double f_min, double f_max, VectorXd &Q, VectorXd &R)
{
    this->dt = timestep;
    this->horizon = horizon;
    this->friction_coeff << friction_coeff, friction_coeff, friction_coeff, friction_coeff; // -x +x -y +y
    this->f_min = f_min;
    this->f_max = f_max;
    // this->Q = Q;
    // this->R = R;

    Ac_dense.resize(5 * num_legs * horizon, 12 * horizon);
    Ac.resize(5 * num_legs * horizon, 12 * horizon);

    Qqp_diag.resize(13 * horizon);
    Rqp_diag.resize(12 * horizon);
    A_qp.resize(13 * horizon, 13);
    B_qp.resize(13 * horizon, 12 * horizon);
    H_dense.resize(12 * horizon, 12 * horizon);
    weighted_B_qp.resize(13 * horizon, 12 * horizon);
    Xref.resize(13 * horizon);
    state_error.resize(13 * horizon);
    weighted_state_error.resize(13 * horizon);

    for (int i = 0; i < horizon; ++i)
    {
        Qqp_diag.segment(i * 13, 13) = Q;
    }
    for (int i = 0; i < horizon; ++i)
    {
        Rqp_diag.segment(i * 12, 12) = R;
    }

    H.resize(12*horizon, 12*horizon);
    q.resize(12*horizon);

    lb.resize(5*num_legs*horizon);
    ub.resize(5*num_legs*horizon);

    init_hessian_pattern();
    calc_constraint_matrix(horizon, this->friction_coeff, Ac);
}

void ConvexMPC::calc_AB_matrices(const VectorXd& body_rpy, const MatrixXd& foot_positions,
                                MatrixXd& A_mat, MatrixXd &B_mat)
{
    // The CoM dynamics can be written as:
    // x_dot = A x + B u
    // where x is the 13-dimensional state vector (r, p, y, x, y, z, r_dot,
    // p_dot, y_dot, vx, vy, vz, -g) constructed from the CoM
    // roll/pitch/yaw/position, and their first order derivatives. 'g' is
    // the gravity constant. u is the 3*num_legs -dimensional input vector
    // ( (fx, fy, fz) for each leg )
    // Construct A matrix (13x13) in the linearized CoM dynamics equation
    // We assume that the input rotation is in X->Y->Z order in the
    // extrinsic/fixed frame, or z->y'->x'' order in the intrinsic frame.

    // calculate A
    cos_yaw = cos(body_rpy(2));
    sin_yaw = sin(body_rpy(2));
    R_z   << cos_yaw, sin_yaw, 0, 
             -sin_yaw,  cos_yaw, 0, 
             0,              0, 1;

    A_mat.setZero();
    A_mat.block<3, 3>(0, 6) = R_z;
    A_mat.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    A_mat(11, 12) = 1;

    // calculate B
    // B (13x(num_legs*3)) contains non_zero elements only in row 6:12.
    inv_inertia = (R_z * robot_params.I_b * R_z.transpose()).inverse();
    B_mat.setZero();
    for (int i = 0; i < num_legs; i++)
    {
        B_mat.block<3, 3>(6, i * 3) = inv_inertia * mors_sys::skew(foot_positions.col(i));
        B_mat.block<3, 3>(9, i * 3) = inv_mass * Eigen::Matrix3d::Identity();
    }
}

void ConvexMPC::calc_discrete_matrices(const MatrixXd& A_mat, const MatrixXd& B_mat, const double planning_timestep,
                                        MatrixXd& Ad_mat, MatrixXd& Bd_mat)
{
    // Calculates the discretized space-time dynamics. Given the dynamics
    // equation:
    // xdot = A x + B u
    // and a timestep dt, we can estimate the snapshot of the state at t +
    // dt by:
    // x(t + dt) = = Ad x + Bd u
    // Using explicit 1st order Euler integration with zero-order hold on u

    Ad_mat.setZero();
    Bd_mat.setZero();
    Ad_mat = MatrixXd::Identity(13, 13) + A_mat * planning_timestep;
    Bd_mat = B_mat * planning_timestep;
}

// Calculate Hessian matrix H and gradient vector q for QP
void ConvexMPC::calc_QP_matrices(const int planning_horizon_steps, const VectorXd& x0, const VectorXd& state_ref,
    const MatrixXd& Ad_mat, const MatrixXd& Bd_mat,
    Sparse_Matrix& H_sparse, VectorXd& q_dense)
{
    const int state_dim = Ad_mat.cols();
    const int action_dim = Bd_mat.cols();

    A_qp.setZero();
    B_qp.setZero();
    H_dense.setZero();
    q_dense.setZero();

    for (int i = 0; i < planning_horizon_steps; i++)
    {
        Xref.segment(i * state_dim, state_dim) = state_ref;
    }

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // calculate A_qp and B_qp
    // A_qp = [A,
    // A^2,
    // A^3,
    // ...
    // A^k]'
    // B_qp = [A^0*B(0),
    // A^1*B(0), B(1),
    // A^2*B(0), A*B(1), B(2),
    // ...
    // A^(k-1)*B(0), A^(k-2)*B(1), A^(k-3)*B(2), ... B(k-1)]
    for (int i = 0; i < planning_horizon_steps; i++)
    {
        if (i == 0)
        {
            A_qp.block(0, 0, state_dim, state_dim) = Ad_mat;
        }
        else
        {
            A_qp.block(state_dim * i, 0, state_dim, state_dim) = A_qp.block(state_dim * (i - 1), 0, state_dim, state_dim) * Ad_mat;
        }
        for (int j = 0; j < i + 1; j++)
        {
            if (i - j == 0)
            {
                B_qp.block(state_dim * i, action_dim * j, state_dim, action_dim) = Bd_mat; // Bd.block(j * state_dim, 0, state_dim, action_dim);
            }
            else
            {
                B_qp.block(state_dim * i, action_dim * j, state_dim, action_dim) =
                    A_qp.block(state_dim * (i - j - 1), 0, state_dim, state_dim) * Bd_mat; // Bd.block(j * state_dim, 0, state_dim, action_dim);
            }
        }
    }
    ///////////////////////////////////////////////////////////////////////
    // calculate hessian
    weighted_B_qp = B_qp.array().colwise() * Qqp_diag.array();
    H_dense.noalias() = 2.0 * (B_qp.transpose() * weighted_B_qp);
    H_dense.diagonal().array() += 2.0 * Rqp_diag.array();
    (void)H_sparse;
    update_hessian_values_from_dense();

    state_error.noalias() = A_qp * x0;
    state_error -= Xref;
    weighted_state_error = Qqp_diag.array() * state_error.array();
    q_dense.noalias() = 2.0 * (B_qp.transpose() * weighted_state_error);
}

void ConvexMPC::calc_constraint_matrix(const int planning_horizon_steps,
                                        const Vector4d& friction_coeff, Sparse_Matrix& Ac_sparse)
{
    Ac_dense.setZero();

    for (int i = 0; i < planning_horizon_steps * num_legs; ++i)
    {
        Ac_dense.block<5, 3>(i * 5, i * 3)
        << -1, 0, friction_coeff(0), // -fx + mu * fz
            1, 0, friction_coeff(1), // fx + mu * fz
            0, -1, friction_coeff(2), // -fy + mu * fz
            0, 1, friction_coeff(3), // fy + mu * fz
            0, 0, 1.0; // fz
    }
    Ac_sparse = Ac_dense.sparseView();
}

void ConvexMPC::calc_constraint_bounds(
                    const int planning_horizon_steps, const vector<int>& contact_state,
                    const double fz_max, const double fz_min,
                    VectorXd& l, VectorXd& u)
{
    for (int i = 0; i < planning_horizon_steps; i++)
    {
        for (int j = 0; j < num_legs; j++)
        {
            if (contact_state[4*i+j] != 0)
                friction_l_u = 1000000;
            else
                friction_l_u = 0.0;

            const int row = (i * num_legs + j) * 5;
            l(row) = 0.0;
            l(row + 1) = 0.0;
            l(row + 2) = 0.0;
            l(row + 3) = 0.0;
            l(row + 4) = fz_min * contact_state[4*i+j];
            u(row) = friction_l_u * contact_state[4*i+j];
            u(row + 1) = friction_l_u * contact_state[4*i+j];
            u(row + 2) = friction_l_u * contact_state[4*i+j];
            u(row + 3) = friction_l_u * contact_state[4*i+j];
            u(row + 4) = fz_max * contact_state[4*i+j];
        }
    }
}

void ConvexMPC::init_hessian_pattern()
{
    std::vector<Eigen::Triplet<double>> triplets;
    const int hessian_dim = H.rows();
    triplets.reserve((hessian_dim * (hessian_dim + 1)) / 2);

    for (int col = 0; col < hessian_dim; ++col)
    {
        for (int row = 0; row <= col; ++row)
        {
            triplets.emplace_back(row, col, 0.0);
        }
    }

    H.setZero();
    H.setFromTriplets(triplets.begin(), triplets.end());
    H.makeCompressed();

    hessian_update_indices.resize(H.nonZeros());
    for (OSQPInt idx = 0; idx < static_cast<OSQPInt>(hessian_update_indices.size()); ++idx)
    {
        hessian_update_indices[idx] = idx;
    }
}

void ConvexMPC::update_hessian_values_from_dense()
{
    int value_idx = 0;
    for (int col = 0; col < H_dense.cols(); ++col)
    {
        for (int row = 0; row <= col; ++row)
        {
            H.valuePtr()[value_idx++] = H_dense(row, col);
        }
    }
}

void ConvexMPC::init_solver()
{
    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setPolish(false);
    // solver.settings()->setAdaptiveRhoInterval(25) ;
    solver.settings()->setAbsoluteTolerance(1e-01);//3);
    solver.settings()->setRelativeTolerance(1e-01);//3);
    solver.settings()->setCheckTermination(1);
    
    solver.settings()->setAlpha(1.5);
    solver.settings()->setMaxIteration(50);
    solver.settings()->setRho(0.1);//1e-3);
    solver.settings()->setSigma(1e-6);
    solver.settings()->setScaledTerimination(1e-6);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(Ac.cols());
    solver.data()->setNumberOfConstraints(Ac.rows());
    solver.data()->setHessianMatrix(H);
    solver.data()->setGradient(q);
    solver.data()->setLinearConstraintsMatrix(Ac);
    solver.data()->setLowerBound(lb);
    solver.data()->setUpperBound(ub);
    // instantiate the solver
    solver.initSolver();

    VectorXd primal_variable_init(horizon * 12);
    VectorXd primal_variable_vec(12);
    primal_variable_vec << 0.0, 0.0,
        (robot_params.M_b * robot_params.g) / 4.0, 0.0, 0.0,
        (robot_params.M_b * robot_params.g) / 4.0, 0.0, 0.0,
        (robot_params.M_b * robot_params.g) / 4.0, 0.0, 0.0,
        (robot_params.M_b * robot_params.g) / 4.0;

    primal_variable_init = primal_variable_vec.replicate(horizon, 1);
    solver.setPrimalVariable(primal_variable_init);

    // solver_inited = true;
}

void ConvexMPC::update_solver()
{
    // update the QP matrices and vectors
    if (!update_solver_data(solver, H, hessian_update_indices, q, lb, ub)) {
        const bool updated = solver.updateHessianMatrix(H)
            && solver.updateGradient(q)
            && solver.updateBounds(lb, ub);
        if (!updated) {
            std::cerr << "[ConvexMPC]: Failed to update solver data\n";
        }
    }
}

VectorXd ConvexMPC::get_contact_forces(const VectorXd& x0, const VectorXd& x_ref,
                                      const MatrixXd& foot_positions, const vector<int>& contact_state)
{
    cur_rpy << x0(0), x0(1), x0(2);

    calc_AB_matrices(cur_rpy, foot_positions, A, B);
    calc_discrete_matrices(A, B, dt, Ad, Bd);
    calc_QP_matrices(horizon, x0, x_ref, Ad, Bd, H, q);
    calc_constraint_bounds(horizon, contact_state, f_max, f_min, lb, ub);

    if (!solver.isInitialized())
        init_solver();
    else
        update_solver();

    solver.solveProblem();

    ref_grf_yaw_aligned = solver.getSolution().head<12>();

    for (int i = 0; i < num_legs; i++)
    {
        ref_grf.segment(i * 3, 3) = ref_grf_yaw_aligned.segment(i * 3, 3);
    }

    return ref_grf;
}   
