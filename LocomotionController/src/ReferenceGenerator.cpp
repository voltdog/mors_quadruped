#include "ReferenceGenerator.hpp"
#include "LowPassFilter.hpp" // Include the low-pass filter implementation
#include <algorithm>
#include <cmath>

// Constructor
ReferenceGenerator::ReferenceGenerator(double dt, double c_freq)
{
    this->c_freq = c_freq;
    this->dt = dt;
    pre_phase_signal = {STANCE, STANCE, STANCE, STANCE};
    foot_pos_global_just_stance.resize(4,3);
    foot_pos_global_just_stance.setZero();
    foot_pos_local_just_stance.resize(4,3);
    foot_pos_local_just_stance.setZero();
    foot_pos_valid_just_stance.fill(false);
    R_body_for_vel.resize(3,3);
    x_ref.resize(13);
    x_ref.setZero();
    ref_yaw_pos = 0.0;
    ref_x_pos = 0.0;
    ref_y_pos = 0.0;
    body_adapt_mode = INCL_ADAPT;

    lpf_x_vel.reconfigureFilter(dt, c_freq);
    lpf_y_vel.reconfigureFilter(dt, c_freq);
    lpf_z_vel.reconfigureFilter(dt, c_freq);
    lpf_pitch_pos.reconfigureFilter(dt, c_freq);
    lpf_z_pos.reconfigureFilter(dt, c_freq);
    lpf_yaw_vel.reconfigureFilter(dt, c_freq);

    // Initialize foot positions in local frame
    double ref_z_pos = 0.0;
    double ref_body_height = 0.23;
    for (int i = 0; i < 4; ++i) {
        foot_pos_local_just_stance(i, Z) = ref_z_pos - ref_body_height;
    }
    for (int i = 0; i < 4; ++i) {
        foot_pos_global_just_stance(i, Z) = -0.038;
    }

    // Initialize reference vector
    x_ref << 0.0, 0.0, 0.0, // orientation
             0.0, 0.0, 0.2, // position
             0.0, 0.0, -0.0, // angular velocity
             0.0, 0.0, 0.0, // linear velocity
             -9.81; // gravity

    ref_body_vel_filtered.resize(3);
    ref_body_vel_directed.resize(3);

    ref_z_pos = 0.0;
    ref_pitch_pos = 0.0;

    test_cnt = 0;

    saved_x_pos = 0;
    saved_y_pos = 0;
    prev_x_vel = 0;
    prev_y_vel = 0;
}

// Destructor
ReferenceGenerator::~ReferenceGenerator() {

}

// Set adaptation mode
void ReferenceGenerator::set_body_adaptation_mode(int mode) {
    this->body_adapt_mode = mode;
}

// Step function
Eigen::VectorXd ReferenceGenerator::step(const std::vector<int>& phase_signal,
                                    const std::vector<Eigen::Vector3d>& foot_pos_global,
                                    const RobotData& robot_cmd,
                                    const RobotData& robot_state) { 

    // Apply low-pass filters to reference velocities
    ref_body_vel_filtered(X) = lpf_x_vel.update(robot_cmd.lin_vel(X));
    ref_body_vel_filtered(Y) = lpf_y_vel.update(robot_cmd.lin_vel(Y));
    ref_body_vel_filtered(Z) = lpf_z_vel.update(robot_cmd.lin_vel(Z));
    ref_body_yaw_vel_filtered = lpf_yaw_vel.update(robot_cmd.ang_vel(Z)); 
    
    // Update reference position
    if (abs(ref_body_vel_filtered(X)) < 0.01 && abs(prev_x_vel) >= 0.01)
        saved_x_pos = robot_state.pos(X);
    if (abs(ref_body_vel_filtered(Y)) < 0.01 && abs(prev_y_vel) >= 0.01)
        saved_y_pos = robot_state.pos(Y);

    ref_x_pos = (abs(ref_body_vel_filtered(X)) < 0.01) ? saved_x_pos : (robot_state.pos(X) + ref_body_vel_filtered(X) * dt);
    ref_y_pos = (abs(ref_body_vel_filtered(Y)) < 0.01) ? saved_y_pos : (robot_state.pos(Y) + ref_body_vel_filtered(Y) * dt);

    update_support_foot_states(phase_signal, foot_pos_global, robot_state);

    // pitch pos adaptation
    bool has_pitch_support = false;
    const double raw_ref_pitch_pos = compute_ref_pitch_pos(phase_signal, has_pitch_support);
    if (body_adapt_mode == INCL_ADAPT) {
        if (has_pitch_support) {
            ref_pitch_pos = lpf_pitch_pos.update(raw_ref_pitch_pos);
        }
    } else {
        ref_pitch_pos = 0.0;
    }
    ref_yaw_pos += ref_body_yaw_vel_filtered * dt;
    
    // Z pos adaptation
    bool has_support = false;
    double support_mean_z_raw = compute_ref_z_pos(phase_signal, has_support);
    if (body_adapt_mode == INCL_ADAPT || body_adapt_mode == HEIGHT_ADAPT) {
        if (has_support) {
            if (ref_pitch_pos > 0.05)
                support_mean_z_raw -= 0.02;
            else if (ref_pitch_pos > 0.05)
                support_mean_z_raw += 0.02;
            support_mean_z = lpf_z_pos.update(support_mean_z_raw);
            ref_z_pos = support_mean_z + robot_cmd.pos(Z);
            
        } else if (std::abs(ref_z_pos) < 1e-9) {
            ref_z_pos = robot_cmd.pos(Z);
        }
    } else {
        ref_z_pos = robot_cmd.pos(Z);
    }

    // Update reference vector
    x_ref << robot_cmd.orientation(X),                  // roll
            ref_pitch_pos + robot_cmd.orientation(Y),   // pitch
            ref_yaw_pos + robot_cmd.orientation(Z),     // yaw
            ref_x_pos + robot_cmd.pos(X),               // pos X
            ref_y_pos + robot_cmd.pos(Y),               // pos Y
            ref_z_pos,                                  // pos Z
            0.0,                                        // angvel roll
            0.0,                                        // angvel pitch
            ref_body_yaw_vel_filtered,                  // angvel yaw
            ref_body_vel_filtered(X),                   // vel X
            ref_body_vel_filtered(Y),                   // vel Y
            robot_cmd.lin_vel(Z),                       // vel Z
             -9.81;

    // Update previous phase signal
    pre_phase_signal = phase_signal;
    prev_x_vel = ref_body_vel_filtered(X);
    prev_y_vel = ref_body_vel_filtered(Y);

    return x_ref;
}

bool ReferenceGenerator::is_support_phase(int phase) const {
    return phase == STANCE || phase == EARLY_CONTACT;
}

bool ReferenceGenerator::is_valid_foot_pos(const Eigen::Vector3d& foot_pos_global,
                                           const RobotData& robot_state) const {
    if (!foot_pos_global.allFinite()) {
        return false;
    }

    const Eigen::Vector3d foot_pos_rel = foot_pos_global - robot_state.pos;
    return foot_pos_rel.norm() < 1.0;
}

Eigen::Vector3d ReferenceGenerator::foot_pos_to_yaw_aligned_local(
    const Eigen::Vector3d& foot_pos_global,
    const RobotData& robot_state) const {
    const double yaw = robot_state.orientation(Z);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);

    Eigen::Matrix3d R_yaw;
    R_yaw << cos_yaw, -sin_yaw, 0.0,
             sin_yaw,  cos_yaw, 0.0,
             0.0,      0.0,     1.0;

    return R_yaw.transpose() * (foot_pos_global - robot_state.pos);
}

void ReferenceGenerator::update_support_foot_states(
    const std::vector<int>& phase_signal,
    const std::vector<Eigen::Vector3d>& foot_pos_global,
    const RobotData& robot_state) {
    const int leg_count = std::min<int>(NUM_LEGS,
                                        std::min(phase_signal.size(), foot_pos_global.size()));
    for (int i = 0; i < leg_count; ++i) {
        if (!is_support_phase(phase_signal[i])) {
            continue;
        }

        if (!is_valid_foot_pos(foot_pos_global[i], robot_state)) {
            continue;
        }

        foot_pos_global_just_stance.row(i) = foot_pos_global[i].transpose();
        foot_pos_local_just_stance.row(i) =
            foot_pos_to_yaw_aligned_local(foot_pos_global[i], robot_state).transpose();
        foot_pos_valid_just_stance[i] = true;
    }
}

// Helper method to compute reference z position
double ReferenceGenerator::compute_ref_z_pos(const std::vector<int>& phase_signal,
                                             bool& has_support) const {
    double mean_z = 0.0;
    int support_count = 0;
    const int leg_count = std::min<int>(NUM_LEGS, phase_signal.size());
    for (int i = 0; i < leg_count; ++i) {
        if (!is_support_phase(phase_signal[i]) || !foot_pos_valid_just_stance[i]) {
            continue;
        }
        mean_z += foot_pos_global_just_stance(i, Z);
        ++support_count;
    }

    has_support = support_count > 0;
    if (!has_support) {
        return 0.0;
    }

    return mean_z / static_cast<double>(support_count);
}

// Helper method to compute reference pitch position
double ReferenceGenerator::compute_ref_pitch_pos(const std::vector<int>& phase_signal,
                                                 bool& has_pitch_support) const {
    Eigen::Vector3d front_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d rear_mean = Eigen::Vector3d::Zero();
    int front_count = 0;
    int rear_count = 0;

    const int front_legs[2] = {R1, L1};
    const int rear_legs[2] = {R2, L2};

    for (int leg_id : front_legs) {
        if (leg_id >= static_cast<int>(phase_signal.size())) {
            continue;
        }
        if (!is_support_phase(phase_signal[leg_id]) || !foot_pos_valid_just_stance[leg_id]) {
            continue;
        }
        front_mean += foot_pos_local_just_stance.row(leg_id).transpose();
        ++front_count;
    }

    for (int leg_id : rear_legs) {
        if (leg_id >= static_cast<int>(phase_signal.size())) {
            continue;
        }
        if (!is_support_phase(phase_signal[leg_id]) || !foot_pos_valid_just_stance[leg_id]) {
            continue;
        }
        rear_mean += foot_pos_local_just_stance.row(leg_id).transpose();
        ++rear_count;
    }

    has_pitch_support = front_count > 0 && rear_count > 0;
    if (!has_pitch_support) {
        return 0.0;
    }

    front_mean /= static_cast<double>(front_count);
    rear_mean /= static_cast<double>(rear_count);

    const double dx = front_mean(X) - rear_mean(X);
    if (std::abs(dx) < 1e-6) {
        has_pitch_support = false;
        return 0.0;
    }

    const double dz = front_mean(Z) - rear_mean(Z);
    return -std::atan2(dz, dx);
}
