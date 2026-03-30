#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include "ReferenceGenerator.hpp"
#include "ContactStateFSM.hpp"
#include "ConvexMpcThread.hpp"
#include "WBICThread.hpp"
#include "SwingController.hpp"
#include "LcmDataExchange.hpp"
#include "SimpleGaitScheduler.hpp"
#include "GaitTransition.hpp"
#include "structs.hpp"
#include "system_functions.hpp"
#include <vbmath.hpp>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;
using namespace YAML;
using namespace std::chrono;

// current time
auto now() 
{
  return std::chrono::steady_clock::now(); 
}


int main() {
    cout << "[LocomotionController]: Starting..." << endl;
    // load config
    string config_address = mors_sys::GetEnv("CONFIGPATH");
    string robot_config_address = config_address + "/robot_config.yaml";

    // robot physical params
    YAML::Node robot_config = YAML::LoadFile(robot_config_address);
    const bool debug_mode = robot_config["debug_mode"] ? robot_config["debug_mode"].as<bool>() : false;
    RobotPhysicalParams robot;
    robot.bx = robot_config["bx"].as<double>(); 
    robot.by = robot_config["by"].as<double>(); 
    robot.m1 = robot_config["m1"].as<double>(); 
    robot.m2 = robot_config["m2"].as<double>(); 
    robot.m3 = robot_config["m3"].as<double>(); 
    robot.l1 = robot_config["l1"].as<double>(); 
    robot.l2 = robot_config["l2"].as<double>(); 
    robot.l3 = robot_config["l3"].as<double>(); 
    robot.d1 = robot_config["d1"].as<double>(); 
    robot.d2 = robot_config["d2"].as<double>(); 
    robot.d3 = robot_config["d3"].as<double>(); 
    robot.l_cz_2 = robot_config["Pc2"][2].as<double>(); 
    robot.l_cx_3 = robot_config["Pc3"][0].as<double>(); 
    robot.g = robot_config["g"].as<double>(); 
    robot.kt = robot_config["kt"].as<double>();
    robot.gear_ratio = robot_config["gear_ratio"].as<double>();
    robot.M_b = robot_config["M"].as<double>(); 
    robot.I_b.resize(3,3);
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
            robot.I_b(i,j) = robot_config["I_b"][i][j].as<double>(); 
        robot.joint_tau_max_array[i] = robot_config["tau_max"][i].as<double>();
        robot.joint_vel_max_array[i] = robot_config["vel_max"][i].as<double>();
    }

    // mpc params
    string mpc_config_address = config_address + "/stance_controller_mpc.yaml";
    YAML::Node mpc_config = YAML::LoadFile(mpc_config_address);
    int horizon = mpc_config["horizon"].as<int>();
    double friction = mpc_config["friction"].as<double>();
    double f_min = mpc_config["f_min"].as<double>();
    double f_max = mpc_config["f_max"].as<double>();
    VectorXd Q_vec(13);
    VectorXd R_vec(12);
    for (int i=0; i<13; i++)
        Q_vec(i) = mpc_config["Q"][i].as<double>(); 
    for (int i=0; i<12; i++)
        R_vec(i) = mpc_config["R"][i].as<double>(); 
    double Q_gain = mpc_config["Q_gain"].as<double>();
    double R_gain = mpc_config["R_gain"].as<double>();
    Q_vec *= Q_gain;
    R_vec *= R_gain;
    cout << "Q_gain: " << Q_gain << " | R_gain: " << R_gain << endl;

    // gait scheduler params
    string gait_scheduler_config_address = config_address + "/gait_scheduler.yaml";
    YAML::Node gait_sched_config = YAML::LoadFile(gait_scheduler_config_address);
    double start_td_detecting = gait_sched_config["start_td_detecting"].as<double>(); 

    // swing controller params
    string swing_controller_config_address = config_address + "/swing_controller.yaml";
    YAML::Node swing_controller_config = YAML::LoadFile(swing_controller_config_address);
    std::array<double, 4> interleave_x; 
    std::array<double, 4> interleave_y;
    for (int i=0; i<4; i++)
    {
        interleave_x[i] = swing_controller_config["interleave_x"][i].as<double>(); 
        interleave_y[i] = swing_controller_config["interleave_y"][i].as<double>(); 
    }
    double dz_near_ground = swing_controller_config["dz_near_ground"].as<double>(); 
    double k1_fsp = swing_controller_config["k1_fsp"].as<double>(); 

    // timesteps duration
    string dt_config_address = config_address + "/timesteps.yaml";
    YAML::Node dt_config = YAML::LoadFile(dt_config_address);
    double module_dt = dt_config["locomotion_controller"].as<double>(); 
    double mpc_dt = dt_config["stance_controller_dt"].as<double>(); 
    double wbic_dt = dt_config["wbic_controller_dt"].as<double>();

    string wbic_config_address = config_address + "/wbic.yaml";
    YAML::Node wbic_config = YAML::LoadFile(wbic_config_address);
    WBICThreadConfig wbic_thread_config;
    wbic_thread_config.timestep = wbic_dt;
    wbic_thread_config.debug_mode = debug_mode;
    wbic_thread_config.Qf_entry = wbic_config["Qf_entry"].as<double>();
    wbic_thread_config.Qa_entry = wbic_config["Qa_entry"].as<double>();
    wbic_thread_config.body_ori_task_kp = wbic_config["body_ori_task_kp"].as<double>();
    wbic_thread_config.body_ori_task_kd = wbic_config["body_ori_task_kd"].as<double>();
    wbic_thread_config.body_pos_task_kp = wbic_config["body_pos_task_kp"].as<double>();
    wbic_thread_config.body_pos_task_kd = wbic_config["body_pos_task_kd"].as<double>();
    wbic_thread_config.tip_pos_task_kp = wbic_config["tip_pos_task_kp"].as<double>();
    wbic_thread_config.tip_pos_task_kd = wbic_config["tip_pos_task_kd"].as<double>();
    wbic_thread_config.joint_kp_stance = wbic_config["joint_kp_stance"].as<double>();
    wbic_thread_config.joint_kd_stance = wbic_config["joint_kd_stance"].as<double>();
    wbic_thread_config.joint_kp_swing = wbic_config["joint_kp_swing"].as<double>();
    wbic_thread_config.joint_kd_swing = wbic_config["joint_kd_swing"].as<double>();
    if (wbic_config["realtime_priority"]) {
        wbic_thread_config.realtime_priority = wbic_config["realtime_priority"].as<int>();
    }
    if (wbic_config["cpu_index"]) {
        wbic_thread_config.cpu_index = wbic_config["cpu_index"].as<int>();
    }
    if (wbic_config["spin_guard_us"]) {
        wbic_thread_config.spin_guard_us = wbic_config["spin_guard_us"].as<int>();
    }

    auto dt = std::chrono::duration<double>(module_dt);//1ms;

    // init LCM
    LCMExchanger lcmExch;
    if (!lcmExch.initialized) {
        cerr << "[LocomotionController]: Failed to initialize LCM exchanger" << endl;
        return -1;
    }
    lcmExch.start_exchanger();

    // init data variables
    RobotData robot_state;
    RobotData robot_cmd, robot_ref;
    LegData leg_state;
    LegData leg_cmd;
    Vector2d robot_pos_offset = Vector2d::Zero();
    double robot_yaw_offset = 0.0;

    // gait generator params
    double t_sw = 0.2;
    double t_st = 0.3;
    std::vector<double> phase_offsets = {0.0, 0.0, 0.0, 0.0};
    bool standing = true;
    bool pre_standing = false;
    std::vector<int> phase_init = {STANCE, STANCE, STANCE, STANCE};
    double stride_height = 0.06;

    SimpleGaitScheduler gait_scheduler(module_dt);
    gait_scheduler.set_gait_params(t_st, t_sw, phase_offsets, phase_init);
    gait_scheduler.setMpcParams(mpc_dt, horizon);
    gait_scheduler.reset();
    gait_scheduler.reset_mpc_table();
    vector<int> gait_table(4 * horizon);
    vector<int> gait_phase(4);
    vector<double> gait_phi(4);

    VectorXd phi;
    vector<double> phi_cur;
    vector<int> phase_signal, pre_phase_signal;
    
    phase_signal.resize(4);
    pre_phase_signal.resize(4);
    phi.resize(4);
    phi.setZero();
    std::vector<Eigen::Vector3d> foot_pos_local(4);
    std::vector<Eigen::Vector3d> foot_pos_global(4);
    std::vector<int> desired_leg_state = {STANCE, STANCE, STANCE, STANCE};
    std::vector<double> leg_phi = {0,0,0,0};

    // init contact state fsm
    ContactStateFSM contact_fsm(start_td_detecting);
    // init reference generator
    int adaptation_type = 0;
    ReferenceGenerator ref_generator(module_dt, 0.5);
    MatrixXd R_body_yaw_align(3,3);
    R_body_yaw_align.setIdentity();
    ref_generator.set_body_adaptation_mode(adaptation_type);

    // swing controller
    VectorXd base_rpy_rate(3);
    VectorXd v(3);
    const double max_leg_length =
        std::sqrt(std::pow(robot.d1 + robot.d2 + robot.d3, 2) +
                  std::pow(robot.l2 + robot.l3, 2));
    SwingController swing_controller(module_dt, 
                                    robot.bx, 
                                    robot.by,  
                                    robot.l1,
                                    max_leg_length,
                                    interleave_x, // сделать правильное чтение данных из конфига
                                    interleave_y,
                                    dz_near_ground, 
                                    k1_fsp);
    
    // other variables
    std::vector<Eigen::Vector3d> p_ref(4), dp_ref(4), ddp_ref(4);
    std::vector<Eigen::Vector3d> p_ref_stance(4);
    std::vector<Eigen::Vector3d> dp_ref_stance(4);
    std::vector<Eigen::Vector3d> ddp_ref_stance(4);
    for (int i = 0; i < 4; i++)
    {
        dp_ref_stance[i].setZero();
        ddp_ref_stance[i].setZero();
    }
    Vector3d ref_body_vel;
    Vector3d base_pos;
    Vector3d base_lin_vel;
    WbcDesiredCommand wbic_command;
    wbic_command.phase_signal = {STANCE, STANCE, STANCE, STANCE};
    wbic_command.active_legs = {true, true, true, true};
    wbic_command.body_cmd.orientation_quaternion << 0.0, 0.0, 0.0, 1.0;

    double t = 0.0;
    vector<bool> active_legs = {true, true, true, true};
    bool enable = false;
    double cos_yaw, sin_yaw;

    VectorXd x0(13);
    VectorXd mpc_cmd(13);
    MatrixXd foot_positions(3, 4);
    VectorXd grf_cmd(12);
    double phi0 = 0.0;

    // Init MPC thread
    ConvexMPCThread mpc_thread;
    mpc_thread.set_physical_params(robot);
    mpc_thread.set_gait_params(t_st, t_sw, phase_offsets, phase_init);
    mpc_thread.set_mpc_params(mpc_dt, horizon, friction, f_min, f_max, Q_vec, R_vec);
    mpc_thread.set_observation_data(robot_state, 
                                    leg_state, 
                                    mpc_cmd, 
                                    R_body_yaw_align, 
                                    enable, 
                                    standing, 
                                    phase_signal, 
                                    phi0,
                                    active_legs);
    
    mpc_thread.start_thread();
    
    // init WBIC thread
    WBICThread wbic_thread;
    wbic_thread.configure(robot, wbic_thread_config);
    wbic_thread.start_thread(lcmExch);

    // gait transition
    GaitTransition gait_transition;
    gait_transition.set_gait_params(t_st, t_sw, phase_offsets, stride_height);
    gait_transition.set_transition_duration(1.0);
    const auto tick_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(dt);
    auto next_tick = now();

    cout << "[LocomotionController]: Started" << endl;

    while(true)
    {
        next_tick += tick_period;

        // ------------------
        // READ LCM
        // ------------------
        robot_cmd = lcmExch.getRobotCmd();
        robot_state = lcmExch.getBodyState();
        leg_state = lcmExch.getLegState();
        lcmExch.get_gait_params(t_st, t_sw, phase_offsets, standing, stride_height);
        enable = lcmExch.get_enable();
        lcmExch.get_active_legs(active_legs);
        adaptation_type = lcmExch.get_adaptation_type();
        
        if (enable == true)
        {
            cos_yaw = cos(robot_state.orientation(Z));
            sin_yaw = sin(robot_state.orientation(Z));
            R_body_yaw_align << cos_yaw, -sin_yaw, 0, 
                                sin_yaw,  cos_yaw, 0,  
                                0,              0, 1;
            // ------------------
            // GAIT SCHEDULER
            // ------------------
            gait_transition.set_gait_params(t_st, t_sw, phase_offsets, stride_height);
            gait_transition.make_transition(t, t_st, t_sw, phase_offsets, stride_height);

            // scheduling
            gait_scheduler.set_gait_params(t_st, t_sw, phase_offsets, phase_init);
            gait_scheduler.step(t, standing, desired_leg_state, leg_phi);
            phase_signal = contact_fsm.step(leg_state.contacts, leg_phi, desired_leg_state);

            // ------------------
            // REFERENCE GENERATOR
            // ------------------
            // transform robot command velocity to WCS
            Vector3d cmd_loc_lin_vel = robot_cmd.lin_vel;
            robot_cmd.lin_vel = R_body_yaw_align * cmd_loc_lin_vel;

            // step reference generator
            ref_generator.set_body_adaptation_mode(adaptation_type);
            mpc_cmd = ref_generator.step(phase_signal, 
                                foot_pos_global,
                                robot_cmd,
                                robot_state);
            // mpc_cmd.segment<2>(3) += robot_pos_offset;
            // mpc_cmd(2) += robot_yaw_offset;
            // ------------------
            // STANCE CONTROLLER
            // ------------------
            phi0 = gait_scheduler.get_phi();
            mpc_thread.set_gait_params(t_st, t_sw, phase_offsets, phase_init);
            mpc_thread.set_observation_data(robot_state, 
                                        leg_state, 
                                        mpc_cmd, 
                                        R_body_yaw_align, 
                                        enable, 
                                        standing, 
                                        phase_signal, 
                                        phi0, 
                                        active_legs);
            grf_cmd = mpc_thread.get_ref_grf();
            
            // when the robot in stance phase the leg tips mustn't move
            if (standing == false) {

                if ((phase_signal[R1] == STANCE || phase_signal[R1] == EARLY_CONTACT) &&
                    (pre_phase_signal[R1] == SWING || pre_phase_signal[R1] == LATE_CONTACT))
                    p_ref_stance[R1] = leg_state.r1_pos;
                if ((phase_signal[L1] == STANCE || phase_signal[L1] == EARLY_CONTACT) &&
                    (pre_phase_signal[L1] == SWING || pre_phase_signal[L1] == LATE_CONTACT))
                    p_ref_stance[L1] = leg_state.l1_pos;
                if ((phase_signal[R2] == STANCE || phase_signal[R2] == EARLY_CONTACT) &&
                    (pre_phase_signal[R2] == SWING || pre_phase_signal[R2] == LATE_CONTACT))
                    p_ref_stance[R2] = leg_state.r2_pos;
                if ((phase_signal[L2] == STANCE || phase_signal[L2] == EARLY_CONTACT) &&
                    (pre_phase_signal[L2] == SWING || pre_phase_signal[L2] == LATE_CONTACT))
                    p_ref_stance[L2] = leg_state.l2_pos;

            }
            else
            {
                if ((standing == true && pre_standing == false))
                    p_ref_stance[R1] = leg_state.r1_pos;
                if ((standing == true && pre_standing == false))
                    p_ref_stance[L1] = leg_state.l1_pos;
                if ((standing == true && pre_standing == false))
                    p_ref_stance[R2] = leg_state.r2_pos;
                if ((standing == true && pre_standing == false))
                    p_ref_stance[L2] = leg_state.l2_pos;
            }
            
            pre_phase_signal[R1] = phase_signal[R1];
            pre_phase_signal[L1] = phase_signal[L1];
            pre_phase_signal[R2] = phase_signal[R2];
            pre_phase_signal[L2] = phase_signal[L2];
            pre_standing = standing;

            // ------------------
            // SWING CONTROLLER
            // ------------------
            base_rpy_rate << robot_state.ang_vel(X), robot_state.ang_vel(Y), robot_state.ang_vel(Z);

            foot_pos_global[0] = leg_state.r1_pos;
            foot_pos_global[1] = leg_state.l1_pos;
            foot_pos_global[2] = leg_state.r2_pos;
            foot_pos_global[3] = leg_state.l2_pos;
 
            // convert data to swing controller format
            ref_body_vel << mpc_cmd(9), mpc_cmd(10), mpc_cmd(11);
            base_pos << robot_state.pos(X), robot_state.pos(Y), robot_state.pos(Z);
            base_lin_vel << robot_state.lin_vel(X), robot_state.lin_vel(Y), robot_state.lin_vel(Z);
 
            swing_controller.set_gait_params(t_sw, t_st, stride_height);
            auto [p_ref, dp_ref, ddp_ref] = swing_controller.step(phase_signal,
                                                                    leg_phi,
                                                                    mpc_cmd(5), 
                                                                    mpc_cmd(8),
                                                                    ref_body_vel,
                                                                    base_pos, 
                                                                    base_lin_vel,
                                                                    base_rpy_rate,
                                                                    R_body_yaw_align,
                                                                    foot_pos_global);

            // convert grf_cmd to convinient format
            if (active_legs[R1] == false) leg_cmd.r1_grf.setZero();
            else leg_cmd.r1_grf = grf_cmd.segment(0, 3);

            if (active_legs[L1] == false) leg_cmd.l1_grf.setZero();
            else leg_cmd.l1_grf = grf_cmd.segment(3, 3);

            if (active_legs[R2] == false) leg_cmd.r2_grf.setZero();
            else leg_cmd.r2_grf = grf_cmd.segment(6, 3);
            
            if (active_legs[L2] == false) leg_cmd.l2_grf.setZero();
            else leg_cmd.l2_grf = grf_cmd.segment(9, 3);

            // choose leg pos, vel and acc depending on swing/stance state
            if (phase_signal[R1] == SWING || phase_signal[R1] == LATE_CONTACT)
            {
                leg_cmd.r1_pos = p_ref[R1];
                leg_cmd.r1_vel = dp_ref[R1];
                leg_cmd.r1_acc = ddp_ref[R1];
            }
            else
            {
                leg_cmd.r1_pos = p_ref_stance[R1];
                leg_cmd.r1_vel = dp_ref_stance[R1];
                leg_cmd.r1_acc.setZero();
            }

            if (phase_signal[L1] == SWING || phase_signal[L1] == LATE_CONTACT)
            {
                leg_cmd.l1_pos = p_ref[L1];
                leg_cmd.l1_vel = dp_ref[L1];
                leg_cmd.l1_acc = ddp_ref[L1];
            }
            else
            {
                leg_cmd.l1_pos = p_ref_stance[L1];
                leg_cmd.l1_vel = dp_ref_stance[L1];
                leg_cmd.l1_acc.setZero();
            }

            if (phase_signal[R2] == SWING || phase_signal[R2] == LATE_CONTACT)
            {
                leg_cmd.r2_pos = p_ref[R2];
                leg_cmd.r2_vel = dp_ref[R2];
                leg_cmd.r2_acc = ddp_ref[R2];
            }
            else
            {
                leg_cmd.r2_pos = p_ref_stance[R2];
                leg_cmd.r2_vel = dp_ref_stance[R2];
                leg_cmd.r2_acc.setZero();
            }

            if (phase_signal[L2] == SWING || phase_signal[L2] == LATE_CONTACT)
            {
                leg_cmd.l2_pos = p_ref[L2];
                leg_cmd.l2_vel = dp_ref[L2];
                leg_cmd.l2_acc = ddp_ref[L2];
            }
            else
            {
                leg_cmd.l2_pos = p_ref_stance[L2];
                leg_cmd.l2_vel = dp_ref_stance[L2];
                leg_cmd.l2_acc.setZero();
            }

            robot_ref.orientation = mpc_cmd.segment(0, 3);
            robot_ref.pos = mpc_cmd.segment(3, 3);
            robot_ref.ang_vel = mpc_cmd.segment(6, 3);
            robot_ref.lin_vel = mpc_cmd.segment(9, 3);
            {
                const Eigen::Quaterniond q_ref =
                    Eigen::AngleAxisd(robot_ref.orientation[Z], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(robot_ref.orientation[Y], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(robot_ref.orientation[X], Eigen::Vector3d::UnitX());
                robot_ref.orientation_quaternion << q_ref.x(), q_ref.y(), q_ref.z(), q_ref.w();
            }

            for (int i = 0; i < NUM_LEGS; ++i)
            {
                wbic_command.phase_signal[i] = phase_signal[i];
                wbic_command.active_legs[i] = active_legs[i];
            }
            wbic_command.body_cmd = robot_ref;
            wbic_command.leg_cmd = leg_cmd;
            wbic_command.locomotion_enabled = true;
            wbic_thread.set_desired_command(wbic_command);

            // ------------------
            // SEND LCM
            // ------------------
            if (debug_mode)
            {
                lcmExch.sendMpcCmd(robot_ref, active_legs);
                lcmExch.sendWbcCmd(robot_ref, leg_cmd);
                lcmExch.sendPhaseSig(phase_signal, leg_phi, t);
            }

            t += module_dt;
        }
        else
        {
            wbic_command.locomotion_enabled = false;
            wbic_thread.set_desired_command(wbic_command);

            // robot_pos_offset = robot_state.pos.segment<2>(0);
            // robot_yaw_offset = robot_state.orientation(Z);
        }

        const auto current_time = now();
        if (current_time < next_tick) {
            std::this_thread::sleep_until(next_tick);
            continue;
        }

        while (next_tick <= current_time) {
            next_tick += tick_period;
        }
    }

    return 0;
}
