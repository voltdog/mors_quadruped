#ifndef _convex_mpc_thread_hpp_
#define _convex_mpc_thread_hpp_

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include <vbmath.hpp>
#include "structs.hpp"
#include "system_functions.hpp"
#include "ConvexMpc.hpp"
#include "SimpleGaitScheduler.hpp"

using namespace Eigen;
using namespace std;

 
class ConvexMPCThread{
public:
    ConvexMPCThread();
    ~ConvexMPCThread();
    auto now();

    void callback();
    void start_thread();

    void set_physical_params(RobotPhysicalParams& robot);
    void set_mpc_params(double timestep, int horizon, double friction_coeff,
                        double f_min, double f_max, VectorXd &Q, VectorXd &R);
    void set_gait_params(double T_st,
                        double T_sw,
                        const std::vector<double>& phase_offsets,
                        const std::vector<int>& phase_init);
    void set_observation_data(const RobotData& robot_state, const LegData& leg_state, const VectorXd& x_ref,
                        const MatrixXd& R_body, const bool& en, const bool& standing, const std::vector<int>& phase_signal,
                        const double& phi0, const vector<bool>& active_legs);
    VectorXd get_ref_grf();

private:
    ConvexMPC mpc;
    SimpleGaitScheduler gait_scheduler;

    RobotData robot_state;
    LegData leg_state;
    VectorXd x_ref;
    bool en;
    bool standing;
    std::vector<int> phase_signal;
    double phi0;

    VectorXd ref_grf;

    VectorXd x0;
    MatrixXd foot_positions;
    vector<int> gait_table;
    double gait_t_st;
    double gait_t_sw;
    vector<double> gait_phase_offsets;
    vector<int> gait_phase_init;
    double module_dt;
    std::chrono::duration<double> dt;
    std::chrono::steady_clock::duration tick_period;
    vector<bool> active_legs;

    RobotPhysicalParams robot;

    std::mutex data_mutex;
    std::mutex ref_grf_mutex;
    std::thread worker_thread;
    std::atomic<bool> running;
};

#endif //_convex_mpc_thread_hpp_
