#include "SwingController.hpp"
#include <Eigen/LU>  // For inverse
#include <algorithm>

SwingController::SwingController(double timestep, double bx, double by, double l1,
                                       double max_leg_length,
                                       const std::array<double, 4>& interleave_x,
                                       const std::array<double, 4>& interleave_y,
                                       double dz_near_ground, double rising_proportion, double k1_fsp)
    : dz_near_ground(dz_near_ground), cnt(4, -1), it_swing(4, 0.0), swing_traj_gen(1.0 / timestep, rising_proportion),
      pre_phase_signal(4, STANCE), p_start(4), p_rise(4), p_finish(4), d_p_start(4, {0.0, 0.0, 0.0}),
      step_planner(4), p0_b(4)
{
    for (int i = 0; i < 4; ++i) {
        step_planner[i] = FootStepPlanner();
        step_planner[i].set_coefficients(k1_fsp);
        step_planner[i].set_max_leg_length(max_leg_length);
    }
    step_planner[R1].set_robot_params(Eigen::Vector3d(bx + l1 + interleave_x[R1], -(by + interleave_y[R1]), 0.0));
    step_planner[L1].set_robot_params(Eigen::Vector3d(bx + l1 + interleave_x[L1],  (by + interleave_y[L1]), 0.0));
    step_planner[R2].set_robot_params(Eigen::Vector3d(-(bx + l1 + interleave_x[R2]), -(by + interleave_y[R2]), 0.0));
    step_planner[L2].set_robot_params(Eigen::Vector3d(-(bx + l1 + interleave_x[L2]),  (by + interleave_y[L2]), 0.0));

    step_planner[R1].set_physical_hip_anchor(Eigen::Vector3d(bx + l1, -by, 0.0));
    step_planner[L1].set_physical_hip_anchor(Eigen::Vector3d(bx + l1,  by, 0.0));
    step_planner[R2].set_physical_hip_anchor(Eigen::Vector3d(-(bx + l1), -by, 0.0));
    step_planner[L2].set_physical_hip_anchor(Eigen::Vector3d(-(bx + l1),  by, 0.0));

    p0_b[R1] = Eigen::Vector3d(bx + l1, -by, 0.0);
    p0_b[L1] = Eigen::Vector3d(bx + l1,  by, 0.0);  
    p0_b[R2] = Eigen::Vector3d(-(bx + l1), -by, 0.0);
    p0_b[L2] = Eigen::Vector3d(-(bx + l1),  by, 0.0);
}

void SwingController::set_gait_params(double t_sw, double t_st, double ref_stride_height) {
    this->t_sw = t_sw;
    this->t_st = t_st;
    this->ref_stride_height = ref_stride_height;
    swing_traj_gen.set_parameters(t_sw, dz_near_ground);
}

void SwingController::set_heightmap(const VisionBasedMap& vision_map)
{
    for (auto& planner : step_planner) {
        planner.set_heightmap(vision_map);
    }
}

std::tuple<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>>
SwingController::step(const std::vector<int>& phase_signal,
                         const std::vector<double>& phi_cur,
                         double ref_body_height,
                         double ref_body_yaw_vel,
                         const Eigen::Vector3d& ref_body_vel,
                         const Eigen::Vector3d& base_pos,
                         const Eigen::Vector3d& base_lin_vel,
                         const Eigen::Vector3d& base_rpy_rate,
                         const Eigen::Matrix3d& R_body,
                         const std::vector<Eigen::Vector3d>& foot_pos_global) {
    
    // compute average legs Z position
    avg_support_foot_z = 0.0;
    support_leg_count = 0;
    for (int i = 0; i < 4; ++i) {
        if (phase_signal[i] == STANCE || phase_signal[i] == EARLY_CONTACT) {
            avg_support_foot_z += foot_pos_global[i](Z);
            ++support_leg_count;
        }
    }
    if (support_leg_count > 0) {
        avg_support_foot_z /= static_cast<double>(support_leg_count);
    }

    for (int i = 0; i < 4; ++i) {
        if (phase_signal[i] == SWING) {
            if (pre_phase_signal[i] == STANCE && phase_signal[i] == SWING) {
                for (int j = 0; j < 3; ++j) p_start[i][j] = foot_pos_global[i](j); 
                cnt[i] = -1;
                swing_traj_gen.reset_offsets();
            }

            const double swing_time_remaining = (1.0 - phi_cur[i]) * t_sw;
            Eigen::Vector3d p_finish_i = step_planner[i].step(base_pos, R_body, base_lin_vel, base_rpy_rate,
                                                               ref_body_vel, ref_body_yaw_vel, 
                                                               swing_time_remaining,
                                                               support_leg_count > 0 ? avg_support_foot_z : p_start[i][Z],
                                                               t_st);
            for (int j = 0; j < 3; ++j) p_finish[i][j] = p_finish_i[j];
            
            Eigen::Vector3d p_hip = Eigen::Vector3d(base_pos(X), base_pos(Y), base_pos(Z)) + R_body * p0_b[i];

            double max_rise_z = p_hip(Z) - 0.03;//  + ref_body_height - 0.05; //
            double p_rise_z = 0.0;
            if ((p_finish[i][Z] - p_start[i][Z]) > 0) {
                p_rise_z = p_finish[i][Z] + ref_stride_height + abs(p_finish[i][Z] - p_start[i][Z]) * 0.2;
            } else {
                p_rise_z = p_start[i][Z] + ref_stride_height;
            }
            p_rise_z = std::min(p_rise_z, max_rise_z); 

            p_rise[i] = {{
                p_start[i][X] + 0.5 * (p_finish[i][X] - p_start[i][X]),
                p_start[i][Y] + 0.5 * (p_finish[i][Y] - p_start[i][Y]),
                p_rise_z
            }};

            it_swing[i] = phi_cur[i] * t_sw;
            cnt[i] += 1;
        } else {
            p_start[i] = {{0.0, 0.0, 0.0}};
            p_rise[i] = {{0.0, 0.0, 0.0}};
            p_finish[i] = {{0.0, 0.0, 0.0}};
            cnt[i] = -1;
        }
    }


    swing_traj_gen.set_parameters(t_sw, dz_near_ground);
    swing_traj_gen.set_points(p_start, p_rise, p_finish, d_p_start);
    auto [x_ref_global, d_p_ref, dd_p_ref] = swing_traj_gen.step(it_swing, cnt, phase_signal);

    std::vector<Eigen::Vector3d> x_ref_glob_out(4), dx_ref(4), ddx_ref(4);
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d x_global(&x_ref_global[i * 3]);
        Eigen::Vector3d dx_global(&d_p_ref[i * 3]);
        Eigen::Vector3d ddx_global(&dd_p_ref[i * 3]);

        x_ref_glob_out[i] = x_global;
        dx_ref[i] = dx_global;
        ddx_ref[i] = ddx_global;
    }

    pre_phase_signal = phase_signal;
    return {x_ref_glob_out, dx_ref, ddx_ref};
}

std::vector<Eigen::Vector3d> SwingController::get_p_finish() const {
    std::vector<Eigen::Vector3d> p_finish_out(NUM_LEGS, Eigen::Vector3d::Zero());
    const int leg_count = std::min<int>(NUM_LEGS, static_cast<int>(p_finish.size()));
    for (int i = 0; i < leg_count; ++i) {
        p_finish_out[i] << p_finish[i][X], p_finish[i][Y], p_finish[i][Z];
    }
    return p_finish_out;
}
