#ifndef _trajectory_generator_hpp_
#define _trajectory_generator_hpp_

#include <Eigen/Dense>
#include <vector>

namespace traj
{

    inline Eigen::VectorXd calc_a(double p0, double pf, double tf, double v0 = 0.0, double vf = 0.0) {
        Eigen::VectorXd a(4);
        a(0) = p0;
        a(1) = v0;
        a(2) = (3.0 / (tf * tf)) * (pf - p0) - 2.0 * v0 / tf - vf / tf;
        a(3) = -(2.0 / (tf * tf * tf)) * (pf - p0) + (v0 + vf) / (tf * tf);
        return a;
    }

    inline std::vector<double> create_qubic_trajectory(double p_start, double p_finish, double tf, double inc) {
        int steps = static_cast<int>(tf / inc);
        std::vector<double> p(steps);
        Eigen::VectorXd a = traj::calc_a(p_start, p_finish, tf, 0.0, 0.0);
        double t = 0.0;
        for (int i = 0; i < steps; ++i) {
            p[i] = a(0) + a(1)*t + a(2)*t*t + a(3)*t*t*t;
            t += inc;
        }
        return p;
    }

    inline std::vector<double> create_qubic_vel_trajectory(double p_start, double p_finish, double tf, double inc) {
        int steps = static_cast<int>(tf / inc);
        std::vector<double> vels(steps);
        Eigen::VectorXd a = traj::calc_a(p_start, p_finish, tf, 0.0, 0.0);
        double t = 0.0;
        for (int i = 0; i < steps; ++i) {
            vels[i] = a(1) + 2*a(2)*t + 3*a(3)*t*t;
            t += inc;
        }
        return vels;
    }

    inline std::vector<std::vector<double>> create_multiple_trajectory(const Eigen::VectorXd& p_start,
                                                                    const Eigen::VectorXd& p_finish,
                                                                    double tf, double inc) {
        std::vector<std::vector<double>> refs;
        for (int i = 0; i < p_start.size(); ++i) {
            refs.push_back(traj::create_qubic_trajectory(p_start(i), p_finish(i), tf, inc));
        }
        return refs;
    }
}

#endif //_trajectory_generator_hpp_
