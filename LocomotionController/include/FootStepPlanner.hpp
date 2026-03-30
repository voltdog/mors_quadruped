#ifndef FOOT_STEP_PLANNER_HPP
#define FOOT_STEP_PLANNER_HPP

#include <deque>
#include <Eigen/Dense>
#include "structs.hpp"

class FootStepPlanner {
public:
    static constexpr double k_raibert = 0.5;

    FootStepPlanner();

    void set_robot_params(const Eigen::Vector3d& p0_b);
    void set_physical_hip_anchor(const Eigen::Vector3d& hip_anchor_b);
    void set_max_leg_length(double max_leg_length);
    void set_coefficients(double k1);
    void set_start_position(const Eigen::Vector3d& base_pos,
                            const Eigen::Vector3d& base_orient);
    Eigen::Vector3d get_hip_location() const;

    Eigen::Vector3d step(const Eigen::Vector3d& body_pos,
                         const Eigen::Matrix3d& R_body,
                         const Eigen::Vector3d& body_lin_vel,
                         const Eigen::Vector3d& body_ang_vel,
                         const Eigen::Vector3d& body_lin_vel_cmd,
                         double body_yaw_vel_cmd,
                         double swing_time_remaining,
                         double prev_p_z,
                         double Tst);

private:
    Eigen::Vector3d p0_b;
    Eigen::Vector3d hip_anchor_b;
    Eigen::Vector3d base_pos;
    Eigen::Vector3d base_orient;
    Eigen::Vector3d hip_location;
    Eigen::Vector3d p_ef_cmd;

    double g;
    double h;
    double k1;
    double k2;
    double max_leg_length;
    

    static constexpr std::size_t dp_hip_window_size = 10;
    std::deque<Eigen::Vector3d> dp_hip_history;
    Eigen::Vector3d dp_hip_sum;
};

#endif // FOOT_STEP_PLANNER_HPP
