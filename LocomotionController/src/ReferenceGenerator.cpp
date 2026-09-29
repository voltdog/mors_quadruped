#include "ReferenceGenerator.hpp"
#include "LowPassFilter.hpp" // Include the low-pass filter implementation
#include <algorithm>
#include <cmath>

// Constructor
ReferenceGenerator::ReferenceGenerator(double dt, double c_freq, double zero_vel_thresh, double foot_valid_radius)
{
    this->c_freq = c_freq;
    this->dt = dt;
    this->zero_vel_thresh = zero_vel_thresh;
    this->foot_valid_radius = foot_valid_radius;
    pre_phase_signal = {STANCE, STANCE, STANCE, STANCE};
    foot_pos_global_just_stance.resize(4,3);
    foot_pos_global_just_stance.setZero();
    foot_pos_local_just_stance.resize(4,3);
    foot_pos_local_just_stance.setZero();
    foot_pos_valid_just_stance.fill(false);
    foot_pos_global_just_swing.resize(4,3);
    foot_pos_global_just_swing.setZero();
    foot_pos_valid_just_swing.fill(false);
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
    lpf_pitch_pos.reconfigureFilter(dt, c_freq*2);
    lpf_z_pos.reconfigureFilter(dt, c_freq*2);
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
                                    const std::vector<Eigen::Vector3d>& foot_pos_finish_global,
                                    const RobotData& robot_cmd,
                                    const RobotData& robot_state) { 
    // Apply low-pass filters to reference velocities
    ref_body_vel_filtered(X) = lpf_x_vel.update(robot_cmd.lin_vel(X));
    ref_body_vel_filtered(Y) = lpf_y_vel.update(robot_cmd.lin_vel(Y));
    ref_body_vel_filtered(Z) = lpf_z_vel.update(robot_cmd.lin_vel(Z));
    ref_body_yaw_vel_filtered = lpf_yaw_vel.update(robot_cmd.ang_vel(Z)); 
    
    // Update reference position
    if (abs(ref_body_vel_filtered(X)) < zero_vel_thresh && abs(prev_x_vel) >= zero_vel_thresh)
        saved_x_pos = robot_state.pos(X);
    if (abs(ref_body_vel_filtered(Y)) < zero_vel_thresh && abs(prev_y_vel) >= zero_vel_thresh)
        saved_y_pos = robot_state.pos(Y);

    ref_x_pos = (abs(ref_body_vel_filtered(X)) < zero_vel_thresh) ? saved_x_pos : (robot_state.pos(X) + ref_body_vel_filtered(X) * dt);
    ref_y_pos = (abs(ref_body_vel_filtered(Y)) < zero_vel_thresh) ? saved_y_pos : (robot_state.pos(Y) + ref_body_vel_filtered(Y) * dt);
    // ref_x_pos += ref_body_vel_filtered(X) * dt;
    // ref_y_pos += ref_body_vel_filtered(Y) * dt;

    update_support_foot_states(phase_signal, foot_pos_global, robot_state);
    update_swing_foot_states(phase_signal, foot_pos_finish_global, robot_state);

    // pitch pos adaptation
    bool has_pitch_support = false;
    const double raw_ref_pitch_pos =
        compute_ref_pitch_pos(phase_signal, robot_state, has_pitch_support);
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
            // if (ref_pitch_pos > 0.05)
            //     support_mean_z_raw -= 0.02;
            // else if (ref_pitch_pos > 0.05)
            //     support_mean_z_raw += 0.02;
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
    return foot_pos_rel.norm() < foot_valid_radius;
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

void ReferenceGenerator::update_swing_foot_states(
    const std::vector<int>& phase_signal,
    const std::vector<Eigen::Vector3d>& foot_pos_finish_global,
    const RobotData& robot_state) {
    const int leg_count = std::min<int>(NUM_LEGS, phase_signal.size());
    for (int i = 0; i < leg_count; ++i) {
        const bool swing_started = (pre_phase_signal[i] == STANCE && phase_signal[i] == SWING);
        if (swing_started) {
            // Start a fresh cache window for this swing and wait for the first valid sample.
            foot_pos_valid_just_swing[i] = false;
            continue;
        }

        if (phase_signal[i] != SWING ||
            foot_pos_valid_just_swing[i] ||
            i >= static_cast<int>(foot_pos_finish_global.size())) {
            continue;
        }

        if (!is_valid_foot_pos(foot_pos_finish_global[i], robot_state)) {
            continue;
        }

        foot_pos_global_just_swing.row(i) = foot_pos_finish_global[i].transpose();
        foot_pos_valid_just_swing[i] = true;
    }
}

// Helper method to compute reference z position
double ReferenceGenerator::compute_ref_z_pos(const std::vector<int>& phase_signal,
                                             bool& has_support) const {
    double mean_z = 0.0;
    int sample_count = 0;
    const int leg_count = std::min<int>(NUM_LEGS, phase_signal.size());
    for (int i = 0; i < leg_count; ++i) {
        if (is_support_phase(phase_signal[i])) {
            if (!foot_pos_valid_just_stance[i]) {
                continue;
            }
            mean_z += foot_pos_global_just_stance(i, Z);
            ++sample_count;
            continue;
        }

        if ((phase_signal[i] != SWING && phase_signal[i] != LATE_CONTACT) ||
            !foot_pos_valid_just_swing[i]) {
            continue;
        }

        mean_z += foot_pos_global_just_swing(i, Z);
        ++sample_count;
    }

    has_support = sample_count > 0;
    if (!has_support) {
        return 0.0;
    }

    return mean_z / static_cast<double>(sample_count);
}

// Helper method to compute reference pitch position
double ReferenceGenerator::compute_ref_pitch_pos(const std::vector<int>& phase_signal,
                                                 const RobotData& robot_state,
                                                 bool& has_pitch_support) const {
    Eigen::Vector3d front_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d rear_mean = Eigen::Vector3d::Zero();
    int front_count = 0;
    int rear_count = 0;

    const int front_legs[2] = {R1, L1};
    const int rear_legs[2] = {R2, L2};

    const auto accumulate_leg_local = [&](int leg_id, Eigen::Vector3d& mean, int& count) {
        if (leg_id >= static_cast<int>(phase_signal.size())) {
            return;
        }

        if (is_support_phase(phase_signal[leg_id])) {
            if (!foot_pos_valid_just_stance[leg_id]) {
                return;
            }
            mean += foot_pos_local_just_stance.row(leg_id).transpose();
            ++count;
            return;
        }

        if ((phase_signal[leg_id] != SWING && phase_signal[leg_id] != LATE_CONTACT) ||
            !foot_pos_valid_just_swing[leg_id]) {
            return;
        }

        const Eigen::Vector3d p_swing_global = foot_pos_global_just_swing.row(leg_id).transpose();
        mean += foot_pos_to_yaw_aligned_local(p_swing_global, robot_state);
        ++count;
    };

    for (int leg_id : front_legs) {
        accumulate_leg_local(leg_id, front_mean, front_count);
    }

    for (int leg_id : rear_legs) {
        accumulate_leg_local(leg_id, rear_mean, rear_count);
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
