#ifndef SWING_LEG_CONTROLLER_HPP
#define SWING_LEG_CONTROLLER_HPP

#include "FootStepPlanner.hpp"
#include "SwingTrajectoryGenerator.hpp"
#include "structs.hpp"
#include <Eigen/Dense>
#include <vector>
#include <array>
#include <tuple>

class SwingController {
public:
    SwingController(double timestep, double bx, double by, double l1,
                       double max_leg_length,
                       const std::array<double, 4>& interleave_x,
                       const std::array<double, 4>& interleave_y,
                       double dz_near_ground, double k1_fsp);

    void set_gait_params(double t_sw, double t_st, double ref_stride_height);

    std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
    step(const std::vector<int>& phase_signal,
         const std::vector<double>& phi_cur,
         double ref_body_height,
         double ref_body_yaw_vel,
         const Eigen::Vector3d& ref_body_vel,
         const Eigen::Vector3d& base_pos,
         const Eigen::Vector3d& base_lin_vel,
         const Eigen::Vector3d& base_rpy_rate,
         const Eigen::Matrix3d& R_body,
         const std::vector<Eigen::Vector3d>& foot_pos_global);

private:
    double dz_near_ground;
    std::vector<int> cnt;
    std::vector<double> it_swing;
    SwingTrajectoryGenerator swing_traj_gen;
    std::vector<int> pre_phase_signal;
    std::vector<std::array<double, 3>> p_start, p_rise, p_finish, d_p_start;
    double t_st, t_sw;
    double ref_stride_height;
    std::vector<FootStepPlanner> step_planner;

    double avg_support_foot_z;
    int support_leg_count;
};

#endif // SWING_LEG_CONTROLLER_HPP
