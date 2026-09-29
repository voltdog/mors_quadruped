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
    this->phase_init_ = {STANCE, STANCE, STANCE, STANCE};
    num_legs_ = static_cast<int>(phase_init_.size());
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
    this->phase_init_ = {STANCE, STANCE, STANCE, STANCE};
    num_legs_ = static_cast<int>(phase_init_.size());
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

    num_legs_ = static_cast<int>(phase_offsets.size());
    full_cycle_period_ = T_st_ + T_sw_;
    duty_factor_ = T_st_ / full_cycle_period_;
    stride_freq = 1.0 / full_cycle_period_;
}
void SimpleGaitScheduler::reset()
{
    normalized_phase_.assign(num_legs_, 0.0);
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
        phi = 0.0;
    }

    phi = std::fmod((phi + stride_freq * dt), 1.0);

    for (int leg = 0; leg < num_legs_; ++leg) 
    {
        phi_[leg] = std::fmod((phi + phase_offsets_[leg]), 1.0);

        if (standing == true)
        {
            if (phi_[leg] < duty_factor_) {
                desired_leg_state_[leg] = STANCE;
            }
            else {
                normalized_phase_[leg] = (phi_[leg] - duty_factor_) / (1.0 - duty_factor_);
            }
        }
        else
        {
            if (phi_[leg] < duty_factor_) {
                desired_leg_state_[leg] = STANCE;
                normalized_phase_[leg] = phi_[leg] / duty_factor_;
            } else {
                desired_leg_state_[leg] = SWING;
                normalized_phase_[leg] = (phi_[leg] - duty_factor_) / (1.0 - duty_factor_);
            }
        }

        leg_state[leg] = desired_leg_state_[leg];
        leg_phase[leg] = normalized_phase_[leg];
    }
    pre_standing = standing;
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
