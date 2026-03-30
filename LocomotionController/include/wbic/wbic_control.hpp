#ifndef _wbic_control_hpp_
#define _wbic_control_hpp_

#include <array>
#include <iostream>

#include <Eigen/Dense>
#include <eiquadprog/eiquadprog-rt.hpp>

#include <vbmath.hpp>

#include "Robot.hpp"
#include "data_types.hpp"
#include "structs.hpp"
#include "wbic/tasks/body_ori_task.hpp"
#include "wbic/tasks/body_pos_task.hpp"
#include "wbic/tasks/tip_pos_task.hpp"

using namespace Eigen;
using namespace std;
namespace wbic_types = mors::wbic;

class WBIC_Control
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr int n_leg = wbic_types::kNumLegs;
    static constexpr int nv = wbic_types::kNv;
    static constexpr int nq = wbic_types::kNq;
    static constexpr int task_dim = wbic_types::kTaskDim;
    static constexpr int max_contact_dim = wbic_types::kMaxContactDim;
    static constexpr int max_support_ineq = wbic_types::kMaxSupportIneq;
    static constexpr int max_qp_vars = wbic_types::kMaxQpVars;
    static constexpr int qp_eq_dim = 6;
    static constexpr int max_tasks = 2 + n_leg;

    static constexpr double ground_fric = 0.3;
    static constexpr double fz_min = 5.0;
    static constexpr double fz_max = 80.0;
    const std::array<double, 3> leg_jnt_range_max{{1.0, 2.0, 3.0}};
    const std::array<double, 3> leg_jnt_range_min{{-1.0, -2.0, -3.0}};

    WBIC_Control();

    void update(Robot& dyn_model,
                const wbic_types::Vector19d& ref_x,
                const wbic_types::Vector18d& ref_dx,
                const wbic_types::Vector18d& ref_ddx,
                const wbic_types::Vector12d& fr_mpc,
                wbic_types::Vector12d& new_q,
                wbic_types::Vector12d& new_dq,
                wbic_types::Vector12d& new_tau);

    void set_q_entries(double Qa_entry, double Qf_entry);
    void set_task_gains(double body_ori_task_kp, double body_ori_task_kd,
                        double body_pos_task_kp, double body_pos_task_kd,
                        double tip_pos_task_kp, double tip_pos_task_kd);
    const wbic_types::Vector12d& get_fr_result() const { return fr_result; }

private:
    using ConfigVector = wbic_types::Vector19d;
    using GenCoordVector = wbic_types::Vector18d;
    using JointVector = wbic_types::Vector12d;
    using ForceVector = wbic_types::Vector12d;
    using BaseVector = wbic_types::Vector6d;
    using TaskVector = wbic_types::Vector3d;
    using SupportVector = wbic_types::Vector24d;
    using NullProjector = wbic_types::Matrix18d;
    using SelectorMatrix = wbic_types::Matrix6_18d;
    using TaskJacobianMatrix = wbic_types::Matrix3_18d;
    using TaskMetricMatrix = wbic_types::Matrix3d;
    using TaskSolverMatrix = wbic_types::Matrix18_3d;
    using ContactJacobianMatrix = wbic_types::Matrix12_18d;
    using ContactMetricMatrix = wbic_types::Matrix12d;
    using ContactSolverMatrix = wbic_types::Matrix18_12d;
    using SupportConstraintMatrix = wbic_types::Matrix24_18d;
    using QpMatrix = wbic_types::Matrix18d;
    using SupportConeMatrix = wbic_types::MatrixRxCd<6, 3>;
    using ActiveTaskArray = std::array<WBCTask*, max_tasks>;
    using TipTaskArray = std::array<TipPosTask, n_leg>;

    void update_task(Robot& dyn_model,
                     const ConfigVector& ref_x,
                     const GenCoordVector& ref_dx,
                     const GenCoordVector& ref_ddx);

    void run_kin_wbc(Robot& dyn_model);

    void run_wbic(Robot& dyn_model,
                  const GenCoordVector& ddq_cmd,
                  const ForceVector& fr_mpc);

    void calc_dynamic_equation_constraint(Robot& dyn_model,
                                          const GenCoordVector& ddq_cmd,
                                          const ForceVector& fr_mpc);

    void calc_support_constraint(Robot& dyn_model,
                                 const ForceVector& fr_mpc);

    void solve_qp(Robot& dyn_model,
                  const GenCoordVector& ddq_cmd,
                  const ForceVector& fr_mpc);

    void solve_joint_tau(Robot& dyn_model);

    void dyn_con_inv(const Eigen::Ref<const MatrixXd>& J);

    double Qa_entry = 1.0;
    double Qf_entry = 1.0;

    eiquadprog::solvers::RtEiquadprog<max_qp_vars, qp_eq_dim, max_support_ineq> qp;

    BodyOriTask body_ori_task;
    BodyPosTask body_pos_task;
    TipTaskArray tip_pos_tasks;
    ActiveTaskArray active_tasks{};
    int active_task_count = 0;

    SelectorMatrix Sf;
    NullProjector I_nv;
    NullProjector Nlast;
    NullProjector task_projector_buf;

    GenCoordVector dq_buf;
    GenCoordVector qdot_buf;
    GenCoordVector qddot_buf;
    GenCoordVector qddot_result;
    GenCoordVector tau_full_result;
    GenCoordVector nle_buf;
    ForceVector fr_result;
    ForceVector fr_mpc_support_buf;
    ForceVector dfr_result_buf;
    ForceVector jcdqd_buf;
    JointVector joint_pos_result;
    JointVector joint_vel_result;
    JointVector joint_tau_result;
    BaseVector da_result_buf;

    TaskVector task_pos_err_buf;
    TaskVector task_vel_des_buf;
    TaskVector task_jtdqd_buf;
    TaskVector task_acc_des_buf;
    TaskVector dq_rhs_buf;
    TaskVector qdot_rhs_buf;
    TaskJacobianMatrix task_jacobian_buf;
    TaskJacobianMatrix Jtpre_buf;
    TaskJacobianMatrix X_task_buf;
    TaskMetricMatrix G_task_buf;
    Eigen::LDLT<TaskMetricMatrix> task_ldlt;

    ContactJacobianMatrix contact_jacobian_buf;
    ContactJacobianMatrix X_contact_buf;
    ContactMetricMatrix G_contact_buf;
    ContactSolverMatrix dyn_con_y_buf;
    ContactSolverMatrix dyn_con_jbar_buf;
    ContactMetricMatrix dyn_con_s_buf;
    ContactMetricMatrix dyn_con_s_inv_buf;
    ContactMetricMatrix dyn_con_identity_buf;
    Eigen::LDLT<Eigen::MatrixXd> contact_ldlt;
    Eigen::LDLT<Eigen::MatrixXd> dyn_con_ldlt;

    QpMatrix qp_hess_buf;
    wbic_types::Vector18d qp_grad_buf;
    wbic_types::Matrix6_18d qp_eq_buf;
    BaseVector qp_eq_target_buf;
    SupportConstraintMatrix qp_ineq_buf;
    SupportVector qp_ineq_target_buf;
    BaseVector qp_eq_offset_buf;
    SupportConstraintMatrix qp_ineq_solver_buf;
    SupportVector qp_ineq_offset_buf;
    wbic_types::Vector18d qp_solution_buf;
    SupportConeMatrix U_support_buf;
    BaseVector u_support_buf;
    int qp_var_count = 0;
    int qp_ineq_count = 0;
    int qp_contact_dim = 0;

    double projector_reg = 1e-8;
    double task_reg = 1e-8;
    NullProjector H_buf;
    Eigen::LDLT<NullProjector> H_ldlt;
    double H_reg = 1e-10;
};

#endif //_wbic_control_hpp_
