#ifndef _robot_hpp_
#define _robot_hpp_

#include <array>
#include <iostream>
#include "system_functions.hpp"
#include "data_types.hpp"
#include <Eigen/Dense>
#include <cmath>
// #include <vbmath.hpp>
#include "structs.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
// #include <pinocchio/algorithm/com.hpp>

// using namespace Eigen;
using namespace std;
namespace wbic_types = mors::wbic;

class Robot{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Robot();
    ~Robot() = default;

    void set_timestep(double dt);
    void BuildPinocchioModel();
    void ComputeForwardKinematics(const VectorXd &q, const VectorXd &v);
    std::vector<Eigen::Vector3d> GetToePositionsInBaseFrame();
    std::vector<Eigen::Vector3d> GetToePositionsInWorldFrame();
    std::vector<Eigen::Matrix3d> GetFootJacobian(const VectorXd &q);
    void ComputeInertiaMatrix(Eigen::VectorXd& q, Eigen::MatrixXd& inertia_matrix);
    void ComputeBiasTerms(Eigen::VectorXd& q, Eigen::VectorXd& q_dot, Eigen::VectorXd& bias_terms);
    Eigen::Vector3d getFootVelocityGlobal(int leg_id);
    bool ComputeIK_CLIK(Eigen::VectorXd &q,
                        const std::vector<Eigen::Vector3d> &desired_pos_base,
                        int max_iter = 50,
                        double tol = 1e-4,
                        double gain = 1.0,
                        double damping = 1e-6);
    void ComputeAllLegDynamics(
                        const Eigen::VectorXd& q,
                        const Eigen::VectorXd& q_dot,
                        std::vector<Eigen::Matrix3d>& M_legs,
                        std::vector<Eigen::Vector3d>& h_legs);

    void update(RobotData& body_state,
                const wbic_types::Vector12d& joint_pos,
                const wbic_types::Vector12d& joint_vel);
    const wbic_types::Matrix3_18d& get_body_ori_jacobian();
    const wbic_types::Matrix3_18d& get_body_pos_jacobian();
    const wbic_types::Vector3d& get_body_ori_jdqd() const;
    const wbic_types::Vector3d& get_body_pos_jdqd() const;
    Vector4d get_body_ori_global();
    Vector3d get_body_pos_global();
    Vector3d get_body_angvel_global();
    Vector3d get_body_vel_global();
    const wbic_types::Matrix3_18d& get_leg_pos_jacobian(int leg_id) const;
    const wbic_types::Vector3d& get_leg_pos_jdqd(int leg_id) const;
    Vector3d get_tip_pos_global(int leg_id);
    Vector3d get_tip_vel_global(int leg_id);
    int get_contact_jacobian_or_none(Eigen::Ref<wbic_types::Matrix12_18d> Jc) const;
    const wbic_types::Vector12d& get_contact_jcdqd_or_none(int& n_support_legs);

    void update_support_states(Vector4i support_states);
    int get_n_support_legs() const;
    bool is_leg_supporting(int leg_id) const;

    const wbic_types::Matrix18d& get_mass_matix() const;
    const wbic_types::Vector18d& get_nle_vector() const;
    const wbic_types::Vector18d& get_tau(const wbic_types::Vector18d& a,
                                         const wbic_types::Vector12d& fr);


    int nq, nv;
    VectorXd q, v;

private:
    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    std::unique_ptr<pinocchio::Data> dynamics_data_;

    std::vector<std::string> ef_frames;
    std::string base_frame;
    std::vector<int> jac_pos_indicies;

    pinocchio::FrameIndex base_frame_id;

    Eigen::Vector3d trans_base;
    Eigen::Vector3d trans_toe;
    Eigen::Vector3d toe_position_body_aligned;

    

    std::array<wbic_types::Matrix3_18d, wbic_types::kNumLegs> J_leg;
    wbic_types::Matrix18d M;
    wbic_types::Vector18d nle;
    Matrix3d R_body;
    wbic_types::Matrix3_18d J_body_ori;
    wbic_types::Matrix3_18d J_body_pos;
    wbic_types::Vector3d Jdqd_body_ori;
    wbic_types::Vector3d Jdqd_body_pos;
    std::array<wbic_types::Vector3d, wbic_types::kNumLegs> Jdqd_leg_cache;
    wbic_types::Matrix6_18d Jd_leg_time_variation_buf;
    wbic_types::Vector12d Jcdqd_support_cache;

    Vector4i support_states;

    wbic_types::Vector18d tau_with_fr;
    wbic_types::Matrix12_18d Jc_all;


    double dt;
};

#endif //_robot_hpp_
