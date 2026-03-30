#include "SwingTrajectoryGenerator.hpp"
#include <cmath>

SwingTrajectoryGenerator::SwingTrajectoryGenerator(double sim_freq)
    : sim_freq(sim_freq), inc(1.0 / sim_freq), dz_near_ground(0.0),
      t_zr(4, 0.0), t_zd(4, 0.0), t_xsw(4, 0.0), t_ysw(4, 0.0),
      a_zr(4, std::vector<double>(6)), a_zd(4, std::vector<double>(6)),
      a_xsw(4, std::vector<double>(6)), a_ysw(4, std::vector<double>(6)),
      it_offset(4, std::vector<double>(4, 0.0)),
      pre_p_rise(4, 0.0), pre_p_desc(4, 0.0),
      pre_px_swing(4, 0.0), pre_px_stance(4, 0.0),
      pre_py_swing(4, 0.0), pre_py_stance(4, 0.0),
      p_start(4), p_rise(4), p_finish(4), d_p_start(4),
      p_x(4, 0.0), p_y(4, 0.0), p_z(4, 0.0),
      p_ref(12, 0.0), d_p_ref(12, 0.0), dd_p_ref(12, 0.0)
{
        tf = 0.0;
}

std::vector<double> SwingTrajectoryGenerator::calc_a(double p0, double pf, double tf,
    double d_p0, double d_pf, double dd_p0, double dd_pf) {
    std::vector<double> a(6);
    a[0] = p0;
    a[1] = d_p0;
    a[2] = dd_p0 / 2.0;
    if (tf != 0.0) {
        a[3] = (20 * pf - 20 * p0 - (8 * d_pf + 12 * d_p0) * tf - (3 * dd_p0 - dd_pf) * tf * tf) / (2 * pow(tf, 3));
        a[4] = (30 * p0 - 30 * pf + (14 * d_pf + 16 * d_p0) * tf + (3 * dd_p0 - 2 * dd_pf) * tf * tf) / (2 * pow(tf, 4));
        a[5] = (12 * pf - 12 * p0 - (6 * d_pf + 6 * d_p0) * tf - (dd_p0 - dd_pf) * tf * tf) / (2 * pow(tf, 5));
    }
    return a;
}

std::tuple<double, double, double> SwingTrajectoryGenerator::calc_spline(const std::vector<double>& a, double t) {
    double t2 = t * t;
    double t3 = t2 * t;
    double t4 = t3 * t;
    double t5 = t4 * t;

    double p = a[0] + a[1] * t + a[2] * t2 + a[3] * t3 + a[4] * t4 + a[5] * t5;
    double dp = a[1] + 2 * a[2] * t + 3 * a[3] * t2 + 4 * a[4] * t3 + 5 * a[5] * t4;
    double ddp = 2 * a[2] + 6 * a[3] * t + 12 * a[4] * t2 + 20 * a[5] * t3;

    return {p, dp, ddp};
}

std::tuple<double, double, double> SwingTrajectoryGenerator::map_z_rising(int leg_num, double it,
    const std::array<double, 3>& p_start, const std::array<double, 3>& p_rise,
    const std::array<double, 3>& d_p_start, double tf, int /*cnt*/) {

    a_zr[leg_num] = calc_a(p_start[Z], p_rise[Z], tf + inc, d_p_start[Z], 0.0, 0.0, 0.0);
    t_zr[leg_num] = it - it_offset[leg_num][Z];
    return calc_spline(a_zr[leg_num], t_zr[leg_num]);
}

std::tuple<double, double, double> SwingTrajectoryGenerator::map_z_descending(int leg_num, double it,
    const std::array<double, 3>& p_rise, const std::array<double, 3>& p_finish,
    double tf, int /*cnt*/, double dp_finish) {

    a_zd[leg_num] = calc_a(p_rise[Z], p_finish[Z], tf, 0.0, dp_finish, 0.0, 0.0);
    t_zd[leg_num] = it - it_offset[leg_num][Z];// - tf;
    return calc_spline(a_zd[leg_num], t_zd[leg_num]);
}

std::tuple<double, double, double> SwingTrajectoryGenerator::map_xy_swing(double it,
    double p_start, double p_finish, double d_p_start, double tf) {

    auto a = calc_a(p_start, p_finish, tf, d_p_start, 0.0, 0.0, 0.0);
    return calc_spline(a, it);
}

void SwingTrajectoryGenerator::set_parameters(double t_swing, double dz_near_ground) {
    tf = t_swing;
    this->dz_near_ground = dz_near_ground;
    cnt_stride = static_cast<int>(t_swing * sim_freq);
}

void SwingTrajectoryGenerator::set_points(const std::vector<std::array<double, 3>>& start,
                                          const std::vector<std::array<double, 3>>& rise,
                                          const std::vector<std::array<double, 3>>& finish,
                                          const std::vector<std::array<double, 3>>& d_start) {
    p_start = start;
    p_rise = rise;
    p_finish = finish;
    d_p_start = d_start;
}

void SwingTrajectoryGenerator::reset_offsets() {
    for (auto& row : it_offset) {
        std::fill(row.begin(), row.end(), 0.0);
    }
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
SwingTrajectoryGenerator::step(const std::vector<double>& it, const std::vector<int>& cnt,
                               const std::vector<int>& leg_phase) {
    for (int i = 0; i < 4; ++i) {
        if (leg_phase[i] == SWING) {
            auto [p_x_val, d_p_x_val, dd_p_x_val] = map_xy_swing(it[i], p_start[i][X], p_finish[i][X], d_p_start[i][X], tf);
            auto [p_y_val, d_p_y_val, dd_p_y_val] = map_xy_swing(it[i], p_start[i][Y], p_finish[i][Y], d_p_start[i][Y], tf);

            double p_z_val, d_p_z_val, dd_p_z_val;
            if (0 <= it[i] && it[i] < tf * rising_proportion) {
                std::tie(p_z_val, d_p_z_val, dd_p_z_val) = map_z_rising(i, it[i], p_start[i], p_rise[i], d_p_start[i], tf * rising_proportion, cnt[i]);
            } else {
                std::tie(p_z_val, d_p_z_val, dd_p_z_val) = map_z_descending(i, (it[i] - tf*rising_proportion), p_rise[i], p_finish[i], tf * (1-rising_proportion), cnt[i], dz_near_ground);
            }

            p_ref[3 * i] = p_x_val;
            p_ref[3 * i + 1] = p_y_val;
            p_ref[3 * i + 2] = p_z_val;

            d_p_ref[3 * i] = d_p_x_val;
            d_p_ref[3 * i + 1] = d_p_y_val;
            d_p_ref[3 * i + 2] = d_p_z_val;

            dd_p_ref[3 * i] = dd_p_x_val;
            dd_p_ref[3 * i + 1] = dd_p_y_val;
            dd_p_ref[3 * i + 2] = dd_p_z_val;
        } else if (leg_phase[i] == LATE_CONTACT) {
            double dz = dz_near_ground;// -0.6;//dz_near_ground < -0.1 ? dz_near_ground : -0.1;
            p_ref[3 * i + 2] += dz * inc;
            d_p_ref[3 * i + 2] = dz;
        }
    }
    return {p_ref, d_p_ref, dd_p_ref};
}
