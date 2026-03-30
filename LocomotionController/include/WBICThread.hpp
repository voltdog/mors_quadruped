#ifndef _wbic_thread_hpp_
#define _wbic_thread_hpp_

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#include "LcmDataExchange.hpp"
#include "Robot.hpp"
#include "structs.hpp"
#include "wbic/wbic_control.hpp"

struct WBICThreadConfig
{
    double timestep = 0.001;
    bool debug_mode = false;
    double Qf_entry = 1.0;
    double Qa_entry = 1.0;
    double body_ori_task_kp = 50.0;
    double body_ori_task_kd = 1.0;
    double body_pos_task_kp = 50.0;
    double body_pos_task_kd = 1.0;
    double tip_pos_task_kp = 100.0;
    double tip_pos_task_kd = 5.0;
    double joint_kp_stance = 0.0;
    double joint_kd_stance = 0.0;
    double joint_kp_swing = 0.0;
    double joint_kd_swing = 0.0;
    int realtime_priority = 75;
    int cpu_index = 3;
    int spin_guard_us = -1;
};

class WBICThread
{
public:
    WBICThread();
    ~WBICThread();

    void configure(const RobotPhysicalParams& robot, const WBICThreadConfig& config);
    void start_thread(LCMExchanger& lcm_exchanger);
    void set_desired_command(const WbcDesiredCommand& desired_command);
    WbcOutputData get_last_output() const;

private:
    using Clock = std::chrono::steady_clock;
    using JointVector = mors::wbic::Vector12d;
    using PhaseVector = mors::wbic::Vector4i;
    using ConfigVector = mors::wbic::Vector19d;
    using GenCoordVector = mors::wbic::Vector18d;

    void callback();
    void publish_zero_command(bool force);
    void wait_until(Clock::time_point deadline) const;
    static Clock::time_point now();

    LCMExchanger* lcm_exchanger = nullptr;

    WBICThreadConfig config_;
    RobotPhysicalParams robot_;
    WBIC_Control wbic_;
    Robot dyn_model_;

    JointVector tau_max_;
    JointVector tau_min_;
    JointVector joint_vel_max_;
    JointVector joint_vel_min_;

    WbcDesiredCommand desired_command_;
    mutable std::mutex desired_mutex_;

    WbcOutputData last_output_;
    mutable std::mutex output_mutex_;

    std::thread worker_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> configured_{false};
    bool zero_command_published_ = false;

    std::chrono::duration<double> dt_{std::chrono::duration<double>::zero()};
    std::chrono::steady_clock::duration tick_period_{std::chrono::steady_clock::duration::zero()};
    std::chrono::steady_clock::duration spin_guard_{std::chrono::steady_clock::duration::zero()};

    JointVector ref_joint_pos_pin;
    JointVector ref_joint_vel_pin;
    JointVector ref_joint_torque_pin;
    JointVector fr_mpc;
    JointVector fr_result;
    JointVector motor_kp;
    JointVector motor_kd;
    JointVector ref_joint_pos_our;
    JointVector ref_joint_vel_our;
    JointVector ref_joint_torque_our;
    ConfigVector ref_x;
    GenCoordVector ref_dx;
    GenCoordVector ref_ddx;
    PhaseVector support_state_pino;
};

#endif //_wbic_thread_hpp_
