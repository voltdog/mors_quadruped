#include "SimpleGaitScheduler.hpp"
#include <Eigen/Dense>
#include <cmath>

SimpleGaitScheduler::SimpleGaitScheduler()
{
    t_offset = 0.0;
    t_start = 0.0;
    pre_leg_state = {STANCE, STANCE, STANCE, STANCE};
    phi = 0.0;
    phi_ = {0.0, 0.0, 0.0, 0.0};
    pre_phi = 0.0;
    // cout << phi[0] << endl;
    this->phase_init_ = {STANCE, STANCE, STANCE, STANCE};//phase_init;
    mpc_leg_state_ = phase_init_;
}

SimpleGaitScheduler::SimpleGaitScheduler(double dt)
{
    t_offset = 0.0;
    t_start = 0.0;
    pre_leg_state = {STANCE, STANCE, STANCE, STANCE};
    phi = 0.0;
    phi_ = {0.0, 0.0, 0.0, 0.0};
    pre_phi = 0.0;

    this->dt = dt;
    // cout << phi[0] << endl;
    this->phase_init_ = {STANCE, STANCE, STANCE, STANCE};//phase_init;
    mpc_leg_state_ = phase_init_;
}

void SimpleGaitScheduler::set_timestep(double dt)
{
    this->dt = dt;
}

void SimpleGaitScheduler::set_gait_params(double T_st,
    double T_sw,
    const std::vector<double>& phase_offsets,
    const std::vector<int>& phase_init)
{
    this->T_st_ = T_st;
    this->T_sw_ = T_sw;
    this->phase_offsets_ = phase_offsets;
    // this->phase_init_ = {STANCE, STANCE, STANCE, STANCE};//phase_init;

    num_legs_ = static_cast<int>(phase_offsets.size());
    full_cycle_period_ = T_st_ + T_sw_;
    duty_factor_ = T_st_ / full_cycle_period_;
    stride_freq = 1.0 / full_cycle_period_;

    // cout << T_st_ << " " << full_cycle_period_ << " " << stride_freq << endl;

    // initial_state_ratio_in_cycle_.resize(num_legs_, duty_factor_);
    // next_leg_state_ = {STANCE, STANCE, STANCE, STANCE};
    // next_leg_state_ = {SWING, SWING, SWING, SWING};

    // for (int leg = 0; leg < num_legs_; ++leg) {
    //     if (phase_init_[leg] == SWING) {
    //         initial_state_ratio_in_cycle_[leg] = 1.0 - duty_factor_;
    //         next_leg_state_[leg] = STANCE;
    //     } else {
    //         initial_state_ratio_in_cycle_[leg] = duty_factor_;
    //         next_leg_state_[leg] = SWING;
    //     }
    // }

    // cout << T_sw << " " << T_st << endl;
    // cout << full_cycle_period_ << " " << duty_factor_ << " " << endl;
    // cout << initial_state_ratio_in_cycle_[0] << " " << initial_state_ratio_in_cycle_[1] << " " << initial_state_ratio_in_cycle_[2] << " " << initial_state_ratio_in_cycle_[3] << endl;
    // cout << "---" << endl;
    // standing_phase = {-1,-1,-1,-1};

    // reset();
}
void SimpleGaitScheduler::reset()
{
    normalized_phase_.assign(num_legs_, 0.0);
    // leg_state_ = phase_init_;
    desired_leg_state_ = phase_init_;
}

void SimpleGaitScheduler::reset_mpc_table()
{
    // mpc_leg_state_ = phase_init_;
}

void SimpleGaitScheduler::step(double t, bool standing, std::vector<int>& leg_state, std::vector<double>& leg_phase)
{
    if (pre_standing == true && standing == false)
    {
        // t_offset = phi;
        phi = 0.0;
        // cout << t_offset << endl;
    }
    phi = std::fmod((phi + stride_freq * dt), 1.0);
    for (int leg = 0; leg < num_legs_; ++leg) 
    {
        
        // double augmented_time = t + phase_offsets_[leg] * full_cycle_period_ - t_offset;
        // double phase_in_full_cycle = std::fmod(augmented_time, full_cycle_period_) / full_cycle_period_;
        // // double augmented_time = t + phase_offsets_[leg] * full_cycle_period_ - t_offset - t_start;
        // // double phase_in_full_cycle = std::fmod(augmented_time / full_cycle_period_, 1.0);
        // double ratio = initial_state_ratio_in_cycle_[leg];
        // normalized_phase_[leg] = phase_in_full_cycle;
        
        // cout << phi[0] << endl;
        // cout << phi[0] << " " << stride_freq << " " << dt << " " << t_offset << endl;
        phi_[leg] = std::fmod((phi + phase_offsets_[leg]), 1.0);
        
        
        
        if (standing == true)
        {
            if (phi_[leg] < duty_factor_) {
                desired_leg_state_[leg] = STANCE;//phase_init_[leg];
                // normalized_phase_[leg] = phase_in_full_cycle / ratio;
                // normalized_phase_[leg] = phi_[leg] / duty_factor_;
            }
            else {
                // desired_leg_state_[leg] = next_leg_state_[leg];
                // normalized_phase_[leg] = (phase_in_full_cycle - ratio) / (1.0 - ratio);
                normalized_phase_[leg] = (phi_[leg] - duty_factor_) / (1.0 - duty_factor_);
            }
        }
        else
        {
            if (phi_[leg] < duty_factor_) {
                desired_leg_state_[leg] = STANCE;//phase_init_[leg];
                normalized_phase_[leg] = phi_[leg] / duty_factor_;
            } else {
                desired_leg_state_[leg] = SWING;//next_leg_state_[leg];
                normalized_phase_[leg] = (phi_[leg] - duty_factor_) / (1.0 - duty_factor_);
            }
        }

        leg_state[leg] = desired_leg_state_[leg];
        leg_phase[leg] = normalized_phase_[leg];
        
        // if (phase_in_full_cycle >= 0.99) {
        //     t_start = t;
        //     cout << "hey" << endl;
        // }
        // pre_leg_state[leg] = leg_state[leg];
    }
    pre_standing = standing;
    // cout << stride_freq << " " << duty_factor_ << endl;
    // cout << phi_[0] << endl;
    // cout << phi[0] << " " << stride_freq << " " << dt << " " << t_offset << endl;
    // cout << leg_state[0] << " " << leg_state[1] << " " << leg_state[2] << " " << leg_state[3] << endl;

}

double SimpleGaitScheduler::get_phi()
{
    return phi;
}

void SimpleGaitScheduler::setMpcParams(double dt_mpc, int n_horizon)
{
    this->dt_mpc_ = dt_mpc;
    this->n_horizon_ = n_horizon;
}

void SimpleGaitScheduler::getMpcTable(double phi0, bool standing, const std::vector<int>& current_leg_state,
                                      const vector<bool>& active_legs, vector<int>& gait_table)
{
    if (static_cast<int>(gait_table.size()) != num_legs_ * n_horizon_) {
        gait_table.resize(num_legs_ * n_horizon_);
    }

    phi = phi0;
    for (int i = 0; i < n_horizon_; i++) {
        phi = std::fmod((phi + stride_freq * dt_mpc_), 1.0);

        for (int leg = 0; leg < num_legs_; leg++) {
            if (active_legs[leg] == false)
            {
                gait_table[i * num_legs_ + leg] = SWING;
            }
            else
            {
                phi_[leg] = std::fmod((phi + phase_offsets_[leg]), 1.0);

                if (standing == true)
                {
                    if ((phi_[leg] < duty_factor_) && (current_leg_state[leg] != SWING))
                        mpc_leg_state_[leg] = STANCE;
                }
                else
                {
                    if (phi_[leg] < duty_factor_) {
                        mpc_leg_state_[leg] = STANCE;
                    } else {
                        mpc_leg_state_[leg] = SWING;
                    }
                }

                if (current_leg_state[leg] == EARLY_CONTACT) {
                    mpc_leg_state_[leg] = STANCE;
                }

                if (i == 0 && current_leg_state[leg] == LATE_CONTACT) {
                    mpc_leg_state_[leg] = SWING;
                }

                gait_table[i * num_legs_ + leg] = mpc_leg_state_[leg];
            }
        }
    }

    pre_standing = standing;
}
