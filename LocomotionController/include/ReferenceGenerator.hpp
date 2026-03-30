#ifndef REFERENCE_GENERATOR_HPP
#define REFERENCE_GENERATOR_HPP

#include <iostream>
#include <Eigen/Dense>
#include <array>
#include <vector>
#include "LowPassFilter.hpp"
#include "structs.hpp"

using namespace std;
using namespace Eigen;

class ReferenceGenerator {
public:

    // Constructor
    ReferenceGenerator(double dt, double c_freq = 1.0);
    ~ReferenceGenerator();

    // Methods
    void set_body_adaptation_mode(int mode);
    Eigen::VectorXd step(const std::vector<int>& phase_signal,
                         const std::vector<Eigen::Vector3d>& foot_pos_global,
                         const RobotData& robot_cmd,
                         const RobotData& robot_state);

private:
    // Parameters
    double c_freq;
    double dt;
    LowPassFilter lpf_x_vel, lpf_y_vel, lpf_z_vel;
    LowPassFilter lpf_pitch_pos, lpf_z_pos, lpf_yaw_vel;

    // State variables
    std::vector<int> pre_phase_signal;
    Eigen::Matrix<double, 4, 3> foot_pos_global_just_stance;
    Eigen::Matrix<double, 4, 3> foot_pos_local_just_stance;
    std::array<bool, NUM_LEGS> foot_pos_valid_just_stance;
    Eigen::VectorXd x_ref;

    // Reference values
    double ref_yaw_pos;
    double ref_x_pos;
    double ref_y_pos;
    double saved_x_pos, saved_y_pos;
    double prev_x_vel, prev_y_vel;
    int body_adapt_mode;

    // Helper methods
    bool is_support_phase(int phase) const;
    bool is_valid_foot_pos(const Eigen::Vector3d& foot_pos_global,
                           const RobotData& robot_state) const;
    Eigen::Vector3d foot_pos_to_yaw_aligned_local(const Eigen::Vector3d& foot_pos_global,
                                                  const RobotData& robot_state) const;
    void update_support_foot_states(const std::vector<int>& phase_signal,
                                    const std::vector<Eigen::Vector3d>& foot_pos_global,
                                    const RobotData& robot_state);
    double compute_ref_z_pos(const std::vector<int>& phase_signal, bool& has_support) const;
    double compute_ref_pitch_pos(const std::vector<int>& phase_signal, bool& has_pitch_support) const;

    Eigen::VectorXd ref_body_vel_filtered, ref_body_vel_directed;
    double ref_body_yaw_vel_filtered;
    MatrixXd R_body_for_vel;

    double ref_z_pos, ref_pitch_pos;
    int test_cnt;
    double support_mean_z;
};

#endif // REFERENCE_GENERATOR_HPP
