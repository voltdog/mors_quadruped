#include "WBICThread.hpp"

#include <cmath>
#include <algorithm>
#include <Eigen/Geometry>

#ifdef __linux__
#if defined(__x86_64__) || defined(__i386__)
#include <immintrin.h>
#endif
#include <pthread.h>
#include <sched.h>
#include <cstring>
#include <unistd.h>
#endif

// RT stuff
namespace {

#ifdef __linux__
void configure_realtime_thread(const char* thread_name,
                               pthread_t handle,
                               int priority,
                               int cpu_index)
{
    bool affinity_ok = false;
    const long cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
    if (cpu_count <= 0) {
        std::cout << '[' << thread_name << "] sysconf(_SC_NPROCESSORS_ONLN) failed\n";
    } else if (cpu_index < 0 || cpu_index >= cpu_count) {
        std::cout << '[' << thread_name << "] requested CPU " << cpu_index
                  << " is out of range 0-" << (cpu_count - 1) << '\n';
    } else {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_index, &cpuset);
        const int affinity_rc = pthread_setaffinity_np(handle, sizeof(cpuset), &cpuset);
        if (affinity_rc != 0) {
            std::cout << '[' << thread_name << "] pthread_setaffinity_np failed: "
                      << std::strerror(affinity_rc) << '\n';
        } else {
            affinity_ok = true;
            std::cout << '[' << thread_name << "] affinity cpu=" << cpu_index << '\n';
        }
    }

    // Avoid unbounded starvation: only switch to SCHED_FIFO when the thread is
    // pinned to a valid core.
    if (!affinity_ok) {
        std::cout << '[' << thread_name
                  << "] realtime scheduling disabled because CPU affinity is unavailable\n";
        return;
    }

    sched_param sp{};
    sp.sched_priority = priority;
    const int sched_rc = pthread_setschedparam(handle, SCHED_FIFO, &sp);
    if (sched_rc != 0) {
        std::cout << '[' << thread_name << "] pthread_setschedparam failed: "
                  << std::strerror(sched_rc) << '\n';
    } else {
        std::cout << '[' << thread_name << "] SCHED_FIFO priority=" << priority << '\n';
    }
}
#endif

Eigen::Quaterniond euler_xyz_to_quaternion(const Eigen::Vector3d& euler)
{
    return Eigen::AngleAxisd(euler[Z], Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(euler[Y], Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(euler[X], Eigen::Vector3d::UnitX());
}

// RT stuff
void cpu_relax()
{
#if defined(__x86_64__) || defined(__i386__)
    _mm_pause();
#else
    std::this_thread::yield();
#endif
}

} // namespace

WBICThread::WBICThread()
{
    desired_command_.phase_signal = {STANCE, STANCE, STANCE, STANCE};
    desired_command_.active_legs = {true, true, true, true};
    desired_command_.body_cmd.orientation_quaternion << 0.0, 0.0, 0.0, 1.0;
}

WBICThread::~WBICThread()
{
    running_ = false;
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }

    if (configured_ && lcm_exchanger != nullptr) {
        publish_zero_command(true);
    }
}

void WBICThread::configure(const RobotPhysicalParams& robot, const WBICThreadConfig& config)
{
    robot_ = robot;
    config_ = config;
    dt_ = std::chrono::duration<double>(config_.timestep);
    tick_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(dt_);
    if (config_.spin_guard_us < 0) {
        // Default to a short spin window. Full-period busy spin causes
        // starvation on some systems and can freeze other control threads.
        const auto default_spin_guard = std::chrono::microseconds(50);
        spin_guard_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(default_spin_guard);
        spin_guard_ = std::min(spin_guard_, tick_period_ / 4);
    } else {
        spin_guard_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::microseconds(std::max(config_.spin_guard_us, 0)));
        spin_guard_ = std::min(spin_guard_, tick_period_);
    }

    tau_max_ << robot_.joint_tau_max_array[0], robot_.joint_tau_max_array[1], robot_.joint_tau_max_array[2],
                robot_.joint_tau_max_array[0], robot_.joint_tau_max_array[1], robot_.joint_tau_max_array[2],
                robot_.joint_tau_max_array[0], robot_.joint_tau_max_array[1], robot_.joint_tau_max_array[2],
                robot_.joint_tau_max_array[0], robot_.joint_tau_max_array[1], robot_.joint_tau_max_array[2];
    tau_min_ = -tau_max_;

    joint_vel_max_ << robot_.joint_vel_max_array[0], robot_.joint_vel_max_array[1], robot_.joint_vel_max_array[2],
                      robot_.joint_vel_max_array[0], robot_.joint_vel_max_array[1], robot_.joint_vel_max_array[2],
                      robot_.joint_vel_max_array[0], robot_.joint_vel_max_array[1], robot_.joint_vel_max_array[2],
                      robot_.joint_vel_max_array[0], robot_.joint_vel_max_array[1], robot_.joint_vel_max_array[2];
    joint_vel_min_ = -joint_vel_max_;

    wbic_.set_q_entries(config_.Qa_entry, config_.Qf_entry);
    wbic_.set_task_gains(config_.body_ori_task_kp,
                         config_.body_ori_task_kd,
                         config_.body_pos_task_kp,
                         config_.body_pos_task_kd,
                         config_.tip_pos_task_kp,
                         config_.tip_pos_task_kd);

    dyn_model_.BuildPinocchioModel();
    configured_ = true;
}

void WBICThread::start_thread(LCMExchanger& lcm_exchanger)
{
    if (!configured_ || running_) {
        return;
    }

    this->lcm_exchanger = &lcm_exchanger;
    running_ = true;
    worker_thread_ = std::thread(&WBICThread::callback, this);

#ifdef __linux__
    configure_realtime_thread("LocomotionController->WBICThread",
                              worker_thread_.native_handle(),
                              config_.realtime_priority,
                              config_.cpu_index);
#endif
}

void WBICThread::set_desired_command(const WbcDesiredCommand& desired_command)
{
    std::lock_guard<std::mutex> lock(desired_mutex_);
    desired_command_ = desired_command;
}

void WBICThread::callback()
{
    RobotData body_state = {};
    LegData leg_state = {};
    ServoStateData servo_state = {};
    WbcDesiredCommand desired_command = {};

    desired_command.phase_signal = {STANCE, STANCE, STANCE, STANCE};
    desired_command.active_legs = {true, true, true, true};

    ref_joint_pos_pin.setZero();
    ref_joint_vel_pin.setZero();
    ref_joint_torque_pin.setZero();
    fr_mpc.setZero();
    fr_result.setZero();
    motor_kp.setZero();
    motor_kd.setZero();
    ref_joint_pos_our.setZero();
    ref_joint_vel_our.setZero();
    ref_joint_torque_our.setZero();
    ref_x.setZero();
    ref_dx.setZero();
    ref_ddx.setZero();
    support_state_pino.setZero();

    bool locomotion_enable = false;
    bool leg_controller_enable = false;
    bool run_wbic = false;
    auto last_overrun_log_time = now();
    int overrun_counter = 0;
    double overrun_max_ms = 0.0;

    auto next_tick = now() + tick_period_;

    while (running_) {
        {
            std::lock_guard<std::mutex> lock(desired_mutex_);
            desired_command = desired_command_;
        }

        lcm_exchanger->get_enable_state(locomotion_enable,
                                        leg_controller_enable);
        lcm_exchanger->get_observation_data(body_state, leg_state, servo_state);


        run_wbic = locomotion_enable && leg_controller_enable;

        if (run_wbic) {
            zero_command_published_ = false;

            // set swing and stance phases based on gait scheduler
            for (int i = 0; i < NUM_LEGS; ++i) {
                const bool support_phase =
                    desired_command.phase_signal[i] == STANCE ||
                    desired_command.phase_signal[i] == EARLY_CONTACT;
                // map from our order to pinnochio
                support_state_pino[i == R1 ? PIN_R1 : i == L1 ? PIN_L1 : i == R2 ? PIN_R2 : PIN_L2] =
                    (desired_command.active_legs[i] && support_phase) ? 1 : 0;
            }

            // update pinocchio model
            dyn_model_.update(body_state, servo_state.position, servo_state.velocity);
            dyn_model_.update_support_states(support_state_pino);

            // just in case let's normalize the quaternion
            Eigen::Vector4d ref_body_quat = desired_command.body_cmd.orientation_quaternion;
            const double ref_quat_norm = ref_body_quat.norm();
            if (ref_quat_norm < 1e-6) {
                const Eigen::Quaterniond q_ref =
                    euler_xyz_to_quaternion(desired_command.body_cmd.orientation);
                ref_body_quat << q_ref.x(), q_ref.y(), q_ref.z(), q_ref.w();
            } else {
                ref_body_quat /= ref_quat_norm;
            }

            // construct desired states: ref_x, ref_dx, ref_ddx, fr_mpc
            ref_x.segment<3>(0) = desired_command.body_cmd.pos;
            ref_x.segment<4>(3) = ref_body_quat;
            ref_x.segment<3>(PIN_START_IDX + PIN_R1 * 3) = desired_command.leg_cmd.r1_pos;
            ref_x.segment<3>(PIN_START_IDX + PIN_L1 * 3) = desired_command.leg_cmd.l1_pos;
            ref_x.segment<3>(PIN_START_IDX + PIN_R2 * 3) = desired_command.leg_cmd.r2_pos;
            ref_x.segment<3>(PIN_START_IDX + PIN_L2 * 3) = desired_command.leg_cmd.l2_pos;

            ref_dx.segment<3>(0) = desired_command.body_cmd.lin_vel;
            ref_dx.segment<3>(3) = desired_command.body_cmd.ang_vel;
            ref_dx.segment<3>(6 + PIN_R1 * 3) = desired_command.leg_cmd.r1_vel;
            ref_dx.segment<3>(6 + PIN_L1 * 3) = desired_command.leg_cmd.l1_vel;
            ref_dx.segment<3>(6 + PIN_R2 * 3) = desired_command.leg_cmd.r2_vel;
            ref_dx.segment<3>(6 + PIN_L2 * 3) = desired_command.leg_cmd.l2_vel;

            ref_ddx.segment<6>(0).setZero();
            ref_ddx.segment<3>(6 + PIN_R1 * 3) = desired_command.leg_cmd.r1_acc;
            ref_ddx.segment<3>(6 + PIN_L1 * 3) = desired_command.leg_cmd.l1_acc;
            ref_ddx.segment<3>(6 + PIN_R2 * 3) = desired_command.leg_cmd.r2_acc;
            ref_ddx.segment<3>(6 + PIN_L2 * 3) = desired_command.leg_cmd.l2_acc;

            fr_mpc.segment<3>(PIN_R1 * 3) = desired_command.leg_cmd.r1_grf;
            fr_mpc.segment<3>(PIN_L1 * 3) = desired_command.leg_cmd.l1_grf;
            fr_mpc.segment<3>(PIN_R2 * 3) = desired_command.leg_cmd.r2_grf;
            fr_mpc.segment<3>(PIN_L2 * 3) = desired_command.leg_cmd.l2_grf;

            // step wbic control
            wbic_.update(dyn_model_,
                            ref_x,
                            ref_dx,
                            ref_ddx,
                            fr_mpc,
                            ref_joint_pos_pin,
                            ref_joint_vel_pin,
                            ref_joint_torque_pin);
            
            // reorder joints from pin to our
            ref_joint_pos_our.segment<3>(R1 * 3) = ref_joint_pos_pin.segment<3>(PIN_R1 * 3);
            ref_joint_pos_our.segment<3>(L1 * 3) = ref_joint_pos_pin.segment<3>(PIN_L1 * 3);
            ref_joint_pos_our.segment<3>(R2 * 3) = ref_joint_pos_pin.segment<3>(PIN_R2 * 3);
            ref_joint_pos_our.segment<3>(L2 * 3) = ref_joint_pos_pin.segment<3>(PIN_L2 * 3);

            ref_joint_vel_our.segment<3>(R1 * 3) = ref_joint_vel_pin.segment<3>(PIN_R1 * 3);
            ref_joint_vel_our.segment<3>(L1 * 3) = ref_joint_vel_pin.segment<3>(PIN_L1 * 3);
            ref_joint_vel_our.segment<3>(R2 * 3) = ref_joint_vel_pin.segment<3>(PIN_R2 * 3);
            ref_joint_vel_our.segment<3>(L2 * 3) = ref_joint_vel_pin.segment<3>(PIN_L2 * 3);

            ref_joint_torque_our.segment<3>(R1 * 3) = ref_joint_torque_pin.segment<3>(PIN_R1 * 3);
            ref_joint_torque_our.segment<3>(L1 * 3) = ref_joint_torque_pin.segment<3>(PIN_L1 * 3);
            ref_joint_torque_our.segment<3>(R2 * 3) = ref_joint_torque_pin.segment<3>(PIN_R2 * 3);
            ref_joint_torque_our.segment<3>(L2 * 3) = ref_joint_torque_pin.segment<3>(PIN_L2 * 3);
            
            // set motor kp and kd gains depending on current leg state (swing or stance)
            for (int i = 0; i < NUM_LEGS; ++i) {
                const bool support_phase = 
                    desired_command.phase_signal[i] == STANCE ||
                    desired_command.phase_signal[i] == EARLY_CONTACT;
                const double kp = support_phase ? config_.joint_kp_stance : config_.joint_kp_swing;
                const double kd = support_phase ? config_.joint_kd_stance : config_.joint_kd_swing;
                motor_kp.segment<3>(i * 3).setConstant(kp);
                motor_kd.segment<3>(i * 3).setConstant(kd);
            }

            // clip torque and then convert to actuator values
            ref_joint_torque_our =
                ref_joint_torque_our.cwiseMin(tau_max_).cwiseMax(tau_min_) *
                robot_.gear_ratio / robot_.kt;
            // clip max joint velocities
            ref_joint_vel_our = ref_joint_vel_our.cwiseMin(joint_vel_max_).cwiseMax(joint_vel_min_);

            // send to actuators
            lcm_exchanger->sendServoCmd(ref_joint_pos_our,
                                        ref_joint_vel_our,
                                        ref_joint_torque_our,
                                        motor_kp,
                                        motor_kd);
            
            // send wbc state if debug_mode=true
            fr_result = wbic_.get_fr_result();
            if (config_.debug_mode) {
                lcm_exchanger->sendWbcState(fr_result);
            }

        } else {
            publish_zero_command(false);
        }

        // wait until desired timestep is over
        const auto current_time = now();
        if (current_time < next_tick) {
            wait_until(next_tick);
            next_tick += tick_period_;
            continue;
        }

        const std::chrono::duration<double, std::milli> lateness{current_time - next_tick};
        if (lateness.count() > 0.05) {
            ++overrun_counter;
            overrun_max_ms = std::max(overrun_max_ms, lateness.count());
        }

        if ((current_time - last_overrun_log_time) >= std::chrono::seconds(1)) {
            if (overrun_counter > 0) {
                std::cout << "[LocomotionController->WBICThread]: Overrun events: "
                          << overrun_counter << ", max lateness: "
                          << overrun_max_ms << " ms\n";
            }
            last_overrun_log_time = current_time;
            overrun_counter = 0;
            overrun_max_ms = 0.0;
        }

        do {
            next_tick += tick_period_;
        } while (next_tick <= current_time);
    }
}

void WBICThread::wait_until(const Clock::time_point deadline) const
{
    if (spin_guard_ <= Clock::duration::zero()) {
        std::this_thread::sleep_until(deadline);
        return;
    }

    const auto sleep_deadline = deadline - spin_guard_;
    if (now() < sleep_deadline) {
        std::this_thread::sleep_until(sleep_deadline);
    }

    while (now() < deadline) {
        cpu_relax();
    }
}

void WBICThread::publish_zero_command(const bool force)
{
    if (lcm_exchanger == nullptr || (!force && zero_command_published_)) {
        return;
    }
    zero_command_published_ = true;

    JointVector12d zero;
    zero.setZero();

    lcm_exchanger->sendWbcState(zero);
    lcm_exchanger->sendServoCmd(zero, zero, zero, zero, zero);
}

WBICThread::Clock::time_point WBICThread::now()
{
    return Clock::now();
}
