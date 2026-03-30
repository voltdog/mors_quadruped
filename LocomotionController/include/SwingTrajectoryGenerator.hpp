#ifndef SWING_TRAJECTORY_GENERATOR_HPP
#define SWING_TRAJECTORY_GENERATOR_HPP

#include <vector>
#include <array>
#include "structs.hpp"

// constexpr int R1 = 0;
// constexpr int L1 = 1;
// constexpr int R2 = 2;
// constexpr int L2 = 3;

// constexpr int X = 0;
// constexpr int Y = 1;
// constexpr int Z = 2;

// constexpr int SWING = 0;
// constexpr int STANCE = 1;
// constexpr int LATE_CONTACT = 2;

class SwingTrajectoryGenerator {
public:
    static constexpr double rising_proportion = 0.25;

    SwingTrajectoryGenerator(double sim_freq = 200.0);

    void set_parameters(double t_swing, double dz_near_ground);
    void set_points(const std::vector<std::array<double, 3>>& p_start,
                    const std::vector<std::array<double, 3>>& p_rise,
                    const std::vector<std::array<double, 3>>& p_finish,
                    const std::vector<std::array<double, 3>>& d_p_start);
    void reset_offsets();
    
    std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
    step(const std::vector<double>& it,
         const std::vector<int>& cnt,
         const std::vector<int>& leg_phase);

private:
    std::vector<double> calc_a(double p0, double pf, double tf,
                               double d_p0 = 0, double d_pf = 0,
                               double dd_p0 = 0, double dd_pf = 0);
    std::tuple<double, double, double> calc_spline(const std::vector<double>& a, double t);

    std::tuple<double, double, double> map_z_rising(int leg_num, double it,
        const std::array<double, 3>& p_start,
        const std::array<double, 3>& p_rise,
        const std::array<double, 3>& d_p_start,
        double tf, int cnt);

    std::tuple<double, double, double> map_z_descending(int leg_num, double it,
        const std::array<double, 3>& p_rise,
        const std::array<double, 3>& p_finish,
        double tf, int cnt, double dp_finish);

    std::tuple<double, double, double> map_xy_swing(double it,
        double p_start, double p_finish,
        double d_p_start, double tf);

    double sim_freq;
    double inc;
    double dz_near_ground;
    int cnt_stride;

    double tf;
    std::vector<double> t_zr, t_zd, t_xsw, t_ysw;
    std::vector<std::vector<double>> a_zr, a_zd, a_xsw, a_ysw;
    std::vector<std::vector<double>> it_offset;

    std::vector<double> pre_p_rise, pre_p_desc;
    std::vector<double> pre_px_swing, pre_px_stance;
    std::vector<double> pre_py_swing, pre_py_stance;

    std::vector<std::array<double, 3>> p_start, p_rise, p_finish, d_p_start;

    std::vector<double> p_x, p_y, p_z;
    std::vector<double> p_ref, d_p_ref, dd_p_ref;
};

#endif // SWING_TRAJECTORY_GENERATOR_HPP
