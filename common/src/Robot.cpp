#include "Robot.hpp"

#include <stdexcept>

Robot::Robot()
{
    ef_frames.resize(4);
    ef_frames[0] = "ef_L1";
    ef_frames[1] = "ef_L2";
    ef_frames[2] = "ef_R1";
    ef_frames[3] = "ef_R2";
    base_frame = "base_link";

    jac_pos_indicies.resize(4);
    jac_pos_indicies[0] = 6;
    jac_pos_indicies[1] = 9;
    jac_pos_indicies[2] = 12;
    jac_pos_indicies[3] = 15;

    dt = 0.002;

}

void Robot::set_timestep(double dt)
{
    this->dt = dt;
}

void Robot::BuildPinocchioModel()
{
    string config_address = mors_sys::GetEnv("CONFIGPATH");
    const std::string urdf_filename = config_address + "/../common/urdf/mors.urdf";

    pinocchio::JointModelFreeFlyer root_joint;

    // std::cout << "root_joint nv: " << root_joint.nv() << std::endl;
    // std::cout << "root_joint nq: " << root_joint.nq() << std::endl;
    // std::cout << "root_joint njoints: " << root_joint.njoints << std::endl;

    pinocchio::urdf::buildModel(urdf_filename, root_joint, model_);
    data_ = std::make_unique<pinocchio::Data>(model_);
    dynamics_data_ = std::make_unique<pinocchio::Data>(model_);

    // std::cout << "Model name: " << model_.name << std::endl;
    // std::cout << "  model nq  = " << model_.nq << " (dimension of configuration space)" << std::endl;
    // std::cout << "  model nv  = " << model_.nv << " (dimension of velocity space)" << std::endl;
    // std::cout << "  model njoints  = " << model_.njoints << " (dimension of velocity space)" << std::endl;
    // std::cout << "  model nframes  = " << model_.nframes << " (dimension of velocity space)" << std::endl;

    // Set model gravity
    model_.gravity.linear() << 0.0, 0.0, -9.81;

    // data_ = pinocchio::Data(model_);
    

    v = Eigen::VectorXd::Zero(model_.nv);
    q = Eigen::VectorXd::Zero(model_.nq);
    q(6) = 1.0;

    ComputeForwardKinematics(q, v);
    pinocchio::ccrba(model_, *data_, q, v);
    auto robot_mass = data_->Ig.mass();
    auto robot_inertia = data_->Ig.inertia();

    // std::cout << "Robot mass :" << robot_mass << std::endl;
    // std::cout << "Robot inertia:\n" << robot_inertia << std::endl;

    // cout << "Joint positions in the world frame:" << std::endl;
    // for (pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id)
    //     std::cout << std::setw(24) << std::left << model_.names[joint_id] << ": " << std::fixed
    //             << std::setprecision(2) << data_->oMi[joint_id].translation().transpose() << std::endl;

    // cout << "\nFrame positions in the world frame:" << std::endl;
    // for (pinocchio::FrameIndex frame_id = 0; frame_id < (pinocchio::FrameIndex)model_.nframes; ++frame_id)
    //     std::cout << std::setw(24) << std::left << model_.frames[frame_id].name << ": " << std::fixed
    //             << std::setprecision(4) << data_->oMf[frame_id].translation().transpose() << std::endl;

    base_frame_id = model_.getFrameId(base_frame);

    nq = model_.nq;
    nv = model_.nv;
    // q.resize(nq);
    // v.resize(nv);
    M.setZero();
    nle.setZero();
    J_body_ori.setZero();
    J_body_pos.setZero();
    Jdqd_body_ori.setZero();
    Jdqd_body_pos.setZero();
    Jd_leg_time_variation_buf.setZero();
    Jcdqd_support_cache.setZero();
    tau_with_fr.setZero();
    Jc_all.setZero();
    for (auto& Jdqd_leg : Jdqd_leg_cache) {
        Jdqd_leg.setZero();
    }
    for (auto& J : J_leg) {
        J.setZero();
    }
}

void Robot::ComputeForwardKinematics(const VectorXd &q, const VectorXd &v) // size(q) = 18
{
    // Вычисление прямой кинематики
    pinocchio::forwardKinematics(model_, *data_, q, v); // добавить сюда еще и q_dot, если нужно
    
    // Обновление позиций фреймов
    pinocchio::updateFramePlacements(model_, *data_);
}

// Метод для получения позиции стопы
std::vector<Eigen::Vector3d> Robot::GetToePositionsInBaseFrame()
{
    std::vector<Eigen::Vector3d> positions;

    for (const auto& frame_name : ef_frames)
    {
        pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);

        trans_base = data_->oMf[base_frame_id].translation();
        trans_toe = data_->oMf[frame_id].translation();
        toe_position_body_aligned = data_->oMf[base_frame_id].rotation().transpose() * (trans_toe - trans_base);
        positions.push_back(toe_position_body_aligned);
    }
    return positions;
}

std::vector<Eigen::Vector3d> Robot::GetToePositionsInWorldFrame()
{
    std::vector<Eigen::Vector3d> positions;

    for (const auto& frame_name : ef_frames)
    {
        pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);

        trans_toe = data_->oMf[frame_id].translation();
        positions.push_back(trans_toe);
    }
    return positions;
}

std::vector<Eigen::Matrix3d> Robot::GetFootJacobian(const VectorXd &q)
{
    std::vector<Eigen::Matrix3d> jacobians;
    int i = 0;

    Matrix3d R = data_->oMf[base_frame_id].rotation();
    pinocchio::computeJointJacobians(model_, *data_, q);

    for (const auto& frame_name : ef_frames)
    {
        pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);
        Eigen::MatrixXd J_full(6, model_.nv);
        Matrix3d Jac = Eigen::Matrix3d::Zero();

        pinocchio::getFrameJacobian(model_, *data_, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_full);
        Jac = R.transpose() * J_full.block(0, jac_pos_indicies[i], 3, 3);
        jacobians.push_back(Jac);

        i++;
    }
    return jacobians;
}

Eigen::Vector3d Robot::getFootVelocityGlobal(int leg_id)
{
    // Get spatial velocity in LOCAL_WORLD_ALIGNED frame
    pinocchio::FrameIndex frame_id = model_.getFrameId(ef_frames[leg_id]);
    Eigen::Vector3d foot_vel_global = pinocchio::getFrameVelocity(model_, *data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED).linear();

    // Eigen::Vector3d foot_pos_base = data_.oMf[leg_id].translation();  // Position of foot in world
    
    // // Compute velocity contribution from base angular motion: ω x r
    // Eigen::Vector3d omega_cross_r = base_angular_velocity_rotated.cross(foot_pos_base);

    // // Compute linear velocity of the foot relative to base
    // Eigen::Vector3d rel_vel = -(foot_vel_global.linear() - omega_cross_r);
    
    // return rel_vel;

    return foot_vel_global;
}

void Robot::ComputeInertiaMatrix(Eigen::VectorXd& q, Eigen::MatrixXd& inertia_matrix)
{
    pinocchio::crba(model_, *data_, q);
    inertia_matrix = data_->M;
}

void Robot::ComputeBiasTerms(Eigen::VectorXd& q, Eigen::VectorXd& q_dot, Eigen::VectorXd& bias_terms)
{
    bias_terms = pinocchio::nonLinearEffects(model_, *data_, q, q_dot);
}

bool Robot::ComputeIK_CLIK(
    Eigen::VectorXd &q,
    const std::vector<Eigen::Vector3d> &desired_pos_base,
    int max_iter,
    double tol,
    double gain,
    double damping)
{
    if (desired_pos_base.size() != 4)
        return false;

    const int task_dim = 12;   // 4 ноги × 3 координаты
    const int dof_dim  = 12;   // 4 ноги × 3 сустава

    for (int iter = 0; iter < max_iter; ++iter)
    {
        // --- FK ---
        Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
        ComputeForwardKinematics(q, v);

        Eigen::Matrix3d R_base = data_->oMf[base_frame_id].rotation();
        Eigen::Vector3d trans_base = data_->oMf[base_frame_id].translation();

        // --- Ошибка (12×1) ---
        Eigen::VectorXd error(task_dim);
        error.setZero();

        // --- Якобиан (12×12) ---
        Eigen::MatrixXd J(task_dim, dof_dim);
        J.setZero();

        pinocchio::computeJointJacobians(model_, *data_, q);

        for (int i = 0; i < 4; ++i)
        {
            pinocchio::FrameIndex frame_id = model_.getFrameId(ef_frames[i]);

            Eigen::Vector3d trans_toe = data_->oMf[frame_id].translation();

            Eigen::Vector3d current_pos_base =
                R_base.transpose() * (trans_toe - trans_base);

            Eigen::Vector3d err_i =
                desired_pos_base[i] - current_pos_base;

            error.segment<3>(3*i) = err_i;

            // --- Якобиан ноги ---
            Eigen::MatrixXd J_full(6, model_.nv);
            pinocchio::getFrameJacobian(
                model_,
                *data_,
                frame_id,
                pinocchio::LOCAL_WORLD_ALIGNED,
                J_full);

            Eigen::Matrix3d J_leg =
                R_base.transpose() *
                J_full.block(0, jac_pos_indicies[i], 3, 3);

            J.block<3,3>(3*i, 3*i) = J_leg;
        }
        // if (!(iter%10))
        //     cout << iter << ": err = " << error.segment<3>(0).transpose() << endl;
        // --- Проверка сходимости ---
        if (error.norm() < tol)
        {   
            // cout << iter << endl;
            return true;
        }
        // --- Damped Least Squares ---
        Eigen::MatrixXd H = J * J.transpose() + damping * Eigen::MatrixXd::Identity(task_dim, task_dim);

        Eigen::VectorXd dq_legs = gain * J.transpose() * H.inverse() * error;

        // --- Формируем полный dq ---
        Eigen::VectorXd dq = Eigen::VectorXd::Zero(model_.nv);

        for (int i = 0; i < 4; ++i)
            dq.segment(jac_pos_indicies[i], 3) = dq_legs.segment<3>(3*i);

        // // --- Интеграция ---
        q = pinocchio::integrate(model_, q, dq*0.5);
    }

    return false;
}

void Robot::ComputeAllLegDynamics(
    const Eigen::VectorXd& q,
    const Eigen::VectorXd& q_dot,
    std::vector<Eigen::Matrix3d>& M_legs,
    std::vector<Eigen::Vector3d>& h_legs)
{
    // M_legs.resize(4);
    // h_legs.resize(4);

    int start_idx = 7; // индекс первого сустава ноги
    std::vector<std::vector<int>> leg_joint_indices(4);
    for (int leg = 0; leg < 4; ++leg) {
        for (int j = 0; j < 3; ++j) {
            leg_joint_indices[leg].push_back(start_idx + leg*3 + j);
        }
    }

    pinocchio::crba(model_, *data_, q);                     // вычисляет M(q) в data.M
    data_->M.triangularView<Eigen::StrictlyLower>() =
        data_->M.transpose().triangularView<Eigen::StrictlyLower>();
    pinocchio::nonLinearEffects(model_, *data_, q, q_dot); // вычисляет nle = C(q,qdot)*qdot + g(q) в data.nle

    for (int leg = 0; leg < 4; leg++) {
        const auto& idx = leg_joint_indices[leg]; // вектор из трёх индексов

        // Матрица инерции ноги 3x3
        Eigen::Matrix3d M_leg = data_->M.block(idx[0], idx[0], 3, 3);
        M_legs[leg] = M_leg;

        // Вектор нелинейных членов ноги 3x1
        Eigen::Vector3d h_leg = data_->nle.segment(idx[0], 3);
        h_legs[leg] = h_leg;
    }
}

void Robot::update(RobotData& body_state,
                   const wbic_types::Vector12d& joint_pos,
                   const wbic_types::Vector12d& joint_vel)
{
    q.segment(0, 3) = body_state.pos;
    q.segment(3, 4) = body_state.orientation_quaternion;
    q.segment(PIN_START_IDX+PIN_R1*3, 3) = joint_pos.segment(R1*3, 3); // order of legs in pinocchio model is L1, L2, R1, R2
    q.segment(PIN_START_IDX+PIN_L1*3, 3) = joint_pos.segment(L1*3, 3);
    q.segment(PIN_START_IDX+PIN_R2*3, 3) = joint_pos.segment(R2*3, 3);
    q.segment(PIN_START_IDX+PIN_L2*3, 3) = joint_pos.segment(L2*3, 3);

    Eigen::Quaterniond q_body(
        body_state.orientation_quaternion[3],
        body_state.orientation_quaternion[0],
        body_state.orientation_quaternion[1],
        body_state.orientation_quaternion[2]);

    R_body = q_body.toRotationMatrix();

    v.segment(0, 3) = R_body.transpose() * body_state.lin_vel; // возможно надо умножить на R_body.T
    v.segment(3, 3) = body_state.ang_vel; // возможно надо умножить на R_body.T 
    v.segment(PIN_START_IDX+PIN_R1*3-1, 3) = joint_vel.segment(R1*3, 3); // order of legs in pinocchio model is L1, L2, R1, R2
    v.segment(PIN_START_IDX+PIN_L1*3-1, 3) = joint_vel.segment(L1*3, 3);
    v.segment(PIN_START_IDX+PIN_R2*3-1, 3) = joint_vel.segment(R2*3, 3);
    v.segment(PIN_START_IDX+PIN_L2*3-1, 3) = joint_vel.segment(L2*3, 3);

    try {
        pinocchio::forwardKinematics(model_, *data_, q, v);
    } catch (const std::exception& ex) {
        throw std::runtime_error(std::string("forwardKinematics failed: ") + ex.what());
    }

    try {
        pinocchio::updateFramePlacements(model_, *data_);
    } catch (const std::exception& ex) {
        throw std::runtime_error(std::string("updateFramePlacements failed: ") + ex.what());
    }

    try {
        pinocchio::crba(model_, *dynamics_data_, q);  // вычисляет M(q) в data.M
    } catch (const std::exception& ex) {
        throw std::runtime_error(
            std::string("crba failed: ") + ex.what() +
            " | q=[" + std::to_string(q.transpose()[0]));
    }
    dynamics_data_->M.triangularView<Eigen::StrictlyLower>() =
        dynamics_data_->M.transpose().triangularView<Eigen::StrictlyLower>();

    try {
        pinocchio::nonLinearEffects(model_, *dynamics_data_, q, v); // вычисляет nle = C(q,qdot)*qdot + g(q) в data.nle
    } catch (const std::exception& ex) {
        throw std::runtime_error(std::string("nonLinearEffects failed: ") + ex.what());
    }

    M = dynamics_data_->M;
    nle = dynamics_data_->nle;

    try {
        pinocchio::computeJointJacobians(model_, *data_, q);
    } catch (const std::exception& ex) {
        throw std::runtime_error(std::string("computeJointJacobians failed: ") + ex.what());
    }

    try {
        pinocchio::computeJointJacobiansTimeVariation(model_, *data_, q, v);
    } catch (const std::exception& ex) {
        throw std::runtime_error(std::string("computeJointJacobiansTimeVariation failed: ") + ex.what());
    }

    // compute joint Jacobians
    int i = 0;

    // cout << "quat: " << q.segment(3, 4).transpose() << endl;

    for (const auto& frame_name : ef_frames)
    {
        pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);
        wbic_types::Matrix6_18d J_full;
        J_full.setZero();

        try {
            pinocchio::getFrameJacobian(
                model_,
                *data_,
                frame_id,
                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                J_full);
        } catch (const std::exception& ex) {
            throw std::runtime_error(std::string("getFrameJacobian failed for frame ") + frame_name + ": " + ex.what());
        }

        J_leg[i] = J_full.topRows(3);

        Jd_leg_time_variation_buf.setZero();
        try {
            pinocchio::getFrameJacobianTimeVariation(
                model_,
                *data_,
                frame_id,
                pinocchio::LOCAL_WORLD_ALIGNED,
                Jd_leg_time_variation_buf);
        } catch (const std::exception& ex) {
            throw std::runtime_error(std::string("getFrameJacobianTimeVariation failed for frame ") + frame_name + ": " + ex.what());
        }
        Jdqd_leg_cache[i].noalias() = Jd_leg_time_variation_buf.topRows(3) * v;

        // cout << "J_leg[" << i << "]: " << endl << J_full.topRows(3) << endl;
        // cout << "---" << endl;

        i++;
    }
    // pinocchio::computeAllTerms(model_, *data_, q, v);
    // M = data_->M.selfadjointView<Eigen::Upper>();
    // nle = data_->nle;


    // cout << "[dyn_model]: H: " << endl << data_->M << endl;
}

const wbic_types::Matrix3_18d& Robot::get_body_ori_jacobian()
{
    J_body_ori.setZero();
    J_body_ori.block(0, 3, 3, 3) = R_body;

    return J_body_ori;
}

const wbic_types::Matrix3_18d& Robot::get_body_pos_jacobian()
{
    J_body_pos.setZero();
    J_body_pos.block(0, 0, 3, 3) = R_body;
    // J_body_pos.block(0, 0, 3, 3).setIdentity();

    return J_body_pos;
}

const wbic_types::Vector3d& Robot::get_body_ori_jdqd() const
{
    return Jdqd_body_ori;
}

const wbic_types::Vector3d& Robot::get_body_pos_jdqd() const
{
    return Jdqd_body_pos;
}

Vector4d Robot::get_body_ori_global()
{
    return q.segment(3, 4);
}

Vector3d Robot::get_body_angvel_global()
{
    return R_body * v.segment(3, 3); //  если в update происходило умножение на R_body.T, то тут множить на R_body
}

Vector3d Robot::get_body_pos_global()
{
    return q.segment(0, 3);
}

Vector3d Robot::get_body_vel_global()
{
    return R_body * v.segment(0, 3); //  если в update происходило умножение на R_body.T, то тут множить на R_body
}

const wbic_types::Matrix3_18d& Robot::get_leg_pos_jacobian(int leg_id) const
{
    return J_leg[leg_id];
}

const wbic_types::Vector3d& Robot::get_leg_pos_jdqd(int leg_id) const
{
    return Jdqd_leg_cache[leg_id];
}

Vector3d Robot::get_tip_pos_global(int leg_id)
{
    pinocchio::FrameIndex frame_id = model_.getFrameId(ef_frames[leg_id]);

    return data_->oMf[frame_id].translation();
}

Vector3d Robot::get_tip_vel_global(int leg_id)
{
    return J_leg[leg_id] * v;
}

int Robot::get_contact_jacobian_or_none(Eigen::Ref<wbic_types::Matrix12_18d> Jc) const
{
    int n_support_legs = get_n_support_legs();
    if (n_support_legs <= 0)
        return 0;

    // MatrixXd Jc(3*n_support_legs, nv);
    int cnt = 0;

    for (int leg = 0; leg < 4; leg++)
    {
        if (support_states[leg] == 1)
        {
            Jc.block(3 * cnt, 0, 3, nv) = J_leg[leg];
            cnt++;
        }
    }

    return cnt;
}

// how to use the function get_contact_jacobian_or_none
// Eigen::MatrixXd Jc(12, nv);
// int n_support = getContactJacobian(Jc);

// if (n_support == 0)
// {
//     // flight phase
// }
// else
// {
//     Eigen::Block<Eigen::MatrixXd> Jc_active =
//         Jc.block(0,0,3*n_support,nv);

//     // используем Jc_active
// }

const wbic_types::Vector12d& Robot::get_contact_jcdqd_or_none(int& n_support_legs)
{
    n_support_legs = get_n_support_legs();
    Jcdqd_support_cache.setZero();
    if (n_support_legs <= 0)
        return Jcdqd_support_cache;

    int cnt = 0;

    for (int leg = 0; leg < 4; leg++)
    {
        if (support_states[leg] == 1)
        {
            Jcdqd_support_cache.segment<3>(3 * cnt) = Jdqd_leg_cache[leg];
            cnt++;
        }
    }
    return Jcdqd_support_cache;
}

void Robot::update_support_states(Vector4i support_states) // 1 - stance, 0 - swing
{
    this->support_states = support_states;
}

int Robot::get_n_support_legs() const
{
    return support_states.sum();
}

bool Robot::is_leg_supporting(int leg_id) const
{
    if (support_states[leg_id] == 1)
        return true;
    else
        return false;
}

const wbic_types::Matrix18d& Robot::get_mass_matix() const
{
    return M;
}

const wbic_types::Vector18d& Robot::get_nle_vector() const
{
    return nle;
}

const wbic_types::Vector18d& Robot::get_tau(const wbic_types::Vector18d& a,
                                            const wbic_types::Vector12d& fr)
{
    // build contact Jacobian
    for (int i = 0; i < 4; i++)
    {
        Jc_all.block(i*3, 0, 3, nv) = J_leg[i];
    }

    // tau = M * qdd + C * qd + g - Jc^T * Fr 
    tau_with_fr = M * a + nle - Jc_all.transpose() * fr;

    return tau_with_fr;
}
