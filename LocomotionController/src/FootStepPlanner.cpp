#include "FootStepPlanner.hpp"
#include <algorithm>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include "system_functions.hpp"

FootStepPlanner::FootStepPlanner()
    : p0_b(0.1655, -0.067, 0.0),
      hip_anchor_b(0.1655, -0.067, 0.0),
      g(9.81),
      k1(0.03),
      max_leg_length(0.3),
      has_vision_map_(false),
      cell_size_(0.015),
      dp_hip_sum(Eigen::Vector3d::Zero())
{
    try {
        std::string config_address = mors_sys::GetEnv("CONFIGPATH");
        config_address += "/heightmap_builder.yaml";
        const YAML::Node cfg = YAML::LoadFile(config_address);
        if (cfg["map"] && cfg["map"]["cell_size"]) {
            const double cell = cfg["map"]["cell_size"].as<double>();
            if (std::isfinite(cell) && cell > 0.0) {
                cell_size_ = cell;
            }
        }
    } catch (const std::exception&) {
        // Keep fallback cell size.
    }
}
 
void FootStepPlanner::set_robot_params(const Eigen::Vector3d& p0_b_) {
    p0_b = p0_b_;
}

void FootStepPlanner::set_physical_hip_anchor(const Eigen::Vector3d& hip_anchor_b_) {
    hip_anchor_b = hip_anchor_b_;
}

void FootStepPlanner::set_max_leg_length(double max_leg_length_) {
    max_leg_length = max_leg_length_;
}

void FootStepPlanner::set_coefficients(double k1_) {
    k1 = k1_;
}

void FootStepPlanner::set_start_position(const Eigen::Vector3d& base_pos_,
                                         const Eigen::Vector3d& base_orient_) {
    base_pos = base_pos_;
    base_orient = base_orient_;
}

void FootStepPlanner::set_heightmap(const VisionBasedMap& vision_map) {
    vision_map_ = vision_map;
    has_vision_map_ = true;
}

Eigen::Vector3d FootStepPlanner::get_hip_location() const { 
    return hip_location;
}

Eigen::Vector3d FootStepPlanner::step(const Eigen::Vector3d& body_pos,
                                      const Eigen::Matrix3d& R_body,
                                      const Eigen::Vector3d& body_lin_vel,
                                      const Eigen::Vector3d& body_ang_vel,
                                      const Eigen::Vector3d& body_lin_vel_cmd,
                                      double body_yaw_vel_cmd,
                                      double swing_time_remaining,
                                      double avg_z_foothold_pos,
                                      double Tst) {

    const Eigen::Vector3d twisting_speed_cmd(0.0, 0.0, body_yaw_vel_cmd);
    const Eigen::AngleAxisd yaw_half_step_rotation(-body_yaw_vel_cmd * Tst * 0.5,
                                                   Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d hip_offset_world = R_body * p0_b;
    const Eigen::Vector3d hip_offset_yaw_corrected_world =
        R_body * (yaw_half_step_rotation * p0_b);

    Eigen::Vector3d p_hip = Eigen::Vector3d(body_pos(X), body_pos(Y), avg_z_foothold_pos) + hip_offset_world;
    p_hip.z() = avg_z_foothold_pos;

    // Rotational contributions are formed in the yaw/body frame, while
    // body_lin_vel is provided in the world frame. Rotate them before summing.
    Eigen::Vector3d p_cross_omega = R_body * body_ang_vel.cross(p0_b);
    Eigen::Vector3d p_cross_omega_cmd = R_body * twisting_speed_cmd.cross(p0_b);
    p_cross_omega.z() = 0.0;
    p_cross_omega_cmd.z() = 0.0;

    Eigen::Vector3d dp_hip = body_lin_vel + p_cross_omega;
    Eigen::Vector3d dp_hip_cmd = body_lin_vel_cmd + p_cross_omega_cmd;
    dp_hip.z() = 0.0;
    dp_hip_cmd.z() = 0.0;

    // dp_hip_history.push_back(dp_hip);
    // dp_hip_sum += dp_hip;
    // if (dp_hip_history.size() > dp_hip_window_size) {
    //     dp_hip_sum -= dp_hip_history.front();
    //     dp_hip_history.pop_front();
    // }
    // dp_hip = dp_hip_sum / static_cast<double>(dp_hip_history.size());

    hip_location = p_hip;
    
    k2 = 0.5 * (body_pos(Z) - avg_z_foothold_pos) / 9.81; // возможно надо вычислять, опираясь на текущие средние значения ног

    Eigen::Vector3d velocity_correction =
        k_raibert * Tst * dp_hip +
        k1 * (dp_hip - dp_hip_cmd) +
        k2 * dp_hip.cross(twisting_speed_cmd);
    velocity_correction.z() = 0.0;

    const double p_rel_max = 0.3;
    velocity_correction.x() = std::clamp(velocity_correction.x(), -p_rel_max, p_rel_max);
    velocity_correction.y() = std::clamp(velocity_correction.y(), -p_rel_max, p_rel_max);

    p_ef_cmd = body_pos + hip_offset_yaw_corrected_world +
                               body_lin_vel_cmd * swing_time_remaining +
                               velocity_correction;
    p_ef_cmd(Z) = avg_z_foothold_pos;

    // Limit the foothold in the horizontal plane relative to the real hip
    // location, not the interleaved nominal foothold offset.
    const Eigen::Vector3d hip_anchor_world = body_pos + R_body * hip_anchor_b;
    Eigen::Vector3d p_rel_body = R_body.transpose() * (p_ef_cmd - hip_anchor_world);
    const double xy_max_sq = std::max(0.0,
                                      max_leg_length * max_leg_length -
                                      p_rel_body(Z) * p_rel_body(Z));
    const double xy_max = std::sqrt(xy_max_sq);
    const double xy_norm = p_rel_body.head<2>().norm();

    if (xy_norm > xy_max && xy_norm > 1e-9) {
        p_rel_body.head<2>() *= xy_max / xy_norm;
        p_ef_cmd = hip_anchor_world + R_body * p_rel_body;
        p_ef_cmd(Z) = avg_z_foothold_pos;
    }

    // search the best position for the footstep on the heightmap and update the command accordingly
    p_ef_cmd_updated = update_footstep_location(p_ef_cmd);

    return p_ef_cmd_updated;
    // return p_ef_cmd;
}

Eigen::Vector3d FootStepPlanner::update_footstep_location(const Eigen::Vector3d& p_ef_cmd_nominal) const
{
    if (!has_vision_map_ || !(cell_size_ > 0.0)) {
        return p_ef_cmd_nominal;
    }

    const int rows = static_cast<int>(vision_map_.heightmap.rows());
    const int cols = static_cast<int>(vision_map_.heightmap.cols());
    if (rows <= 0 || cols <= 0) {
        return p_ef_cmd_nominal;
    }

    const int center_row =
        static_cast<int>(std::floor((p_ef_cmd_nominal(Y) - static_cast<double>(vision_map_.origin_y)) / cell_size_)) +
        rows / 2;
    const int center_col =
        static_cast<int>(std::floor((p_ef_cmd_nominal(X) - static_cast<double>(vision_map_.origin_x)) / cell_size_)) +
        cols / 2;

    int selected_row = center_row;
    int selected_col = center_col;
    if (!search_foothold(center_row, center_col, selected_row, selected_col)) {
        return p_ef_cmd_nominal;
    }

    Eigen::Vector3d foothold = p_ef_cmd_nominal;
    const double start_x =
        static_cast<double>(vision_map_.origin_x) - 0.5 * static_cast<double>(cols) * cell_size_;
    const double start_y =
        static_cast<double>(vision_map_.origin_y) - 0.5 * static_cast<double>(rows) * cell_size_;

    foothold(X) = start_x + (static_cast<double>(selected_col) + 0.5) * cell_size_;
    foothold(Y) = start_y + (static_cast<double>(selected_row) + 0.5) * cell_size_;
    foothold(Z) = static_cast<double>(vision_map_.heightmap(selected_row, selected_col));// + 0.005; // + 0.015;//- 0.02;// 
    return foothold;
}

bool FootStepPlanner::search_foothold(int center_row,
                                      int center_col,
                                      int& selected_row,
                                      int& selected_col) const
{
    const int rows = static_cast<int>(vision_map_.heightmap.rows());
    const int cols = static_cast<int>(vision_map_.heightmap.cols());
    for (const auto& search_idx : search_idxs) {
        const int row = center_row + search_idx[1];
        const int col = center_col + search_idx[0];
        if (row < 0 || row >= rows || col < 0 || col >= cols) {
            continue;
        }

        const uint8_t traversability = vision_map_.traversability(row, col);
        const float height = vision_map_.heightmap(row, col);
        if (traversability == 0u && std::isfinite(static_cast<double>(height))) {
            selected_row = row;
            selected_col = col;
            return true;
        }
    }

    return false;
}
