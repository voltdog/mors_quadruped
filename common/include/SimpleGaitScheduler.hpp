#ifndef SIMPLE_GAIT_SCHEDULER_HPP
#define SIMPLE_GAIT_SCHEDULER_HPP

#include <Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <cmath>
#include <vector>


using namespace std;
using namespace Eigen;
// using namespace YAML;

#define    SWING         0
#define    STANCE        1
#define    LATE_CONTACT  2
#define    EARLY_CONTACT 3

class SimpleGaitScheduler {
public:
    SimpleGaitScheduler();
    SimpleGaitScheduler(double dt);

    void set_timestep(double dt);
    void set_gait_params(double T_st,
                        double T_sw,
                        const std::vector<double>& phase_offsets,
                        const std::vector<int>& phase_init);

    void reset();
    void reset_mpc_table();
    void step(double t, bool standing, std::vector<int>& leg_state, std::vector<double>& leg_phase);
    void setMpcParams(double dt_mpc, int n_horizon);
    void getMpcTable(double phi0, bool standing, const std::vector<int>& current_leg_state,
                     const vector<bool>& active_legs, vector<int>& gait_table);
    double get_phi();

private:
    double T_st_;
    double T_sw_;
    double full_cycle_period_;
    double duty_factor_;
    double touchdown_detection_threshold_;
    double contact_threshold_;
    double dt_mpc_, dt;
    int n_horizon_;

    int num_legs_;
    std::vector<double> phase_offsets_;
    std::vector<int> phase_init_;
    std::vector<double> initial_state_ratio_in_cycle_;
    std::vector<int> next_leg_state_;

    std::vector<int> leg_state_, pre_leg_state;
    std::vector<int> mpc_leg_state_;
    std::vector<int> desired_leg_state_;
    std::vector<double> normalized_phase_;
    // vector<bool> standing_phase;
    bool pre_standing;
    double t_offset, t_start;
    double stride_freq;
    double phi, pre_phi;
    std::vector<double> phi_;
};

#endif // SIMPLE_GAIT_SCHEDULER_HPP
