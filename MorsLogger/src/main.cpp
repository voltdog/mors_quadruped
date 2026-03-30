#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include "lcm_data_exchange_ml.hpp"
#include "structs.hpp"
#include "csv_maintaner.hpp"
#include <vbmath.hpp>
#include <yaml-cpp/yaml.h>
#include "system_functions.hpp"

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
    cout << "MORS Logger starting..." << endl;
    // load config
    string config_address = mors_sys::GetEnv("CONFIGPATH");
    string robot_config_address = config_address + "/robot_config.yaml";
    YAML::Node robot_config = YAML::LoadFile(robot_config_address);
    const bool debug_mode = robot_config["debug_mode"] ? robot_config["debug_mode"].as<bool>() : false;

    string dt_config_address = config_address + "/timesteps.yaml";
    YAML::Node dt_config = YAML::LoadFile(dt_config_address);
    double module_dt = dt_config["mors_logger_dt"].as<double>(); 

    auto dt = std::chrono::duration<double>(module_dt);//1ms;

    // define lcm exchanger
    LCMExchanger lcmExch(debug_mode);
    lcmExch.start_exchanger();
    std::this_thread::sleep_for(5ms);

    // define lcm data
    LegData wbc_leg_cmd;
    RobotData wbc_robot_cmd, mpc_robot_cmd;
    LegData leg_cmd;
    ImuData imu_data;
    ServoData servo_state;
    ServoData servo_cmd;
    Odometry odometry;
    ServoData servo_state_filt;
    LegData leg_state;
    RobotData body_state;
    LegData leg_state_check;
    RobotData body_state_check;
    RobotData body_cmd;
    Vector4i phase;
    Vector4d phi;
    vector<bool> contact_states(4);

    // bool leg_controller_enable, leg_controller_reset;
    // bool locomotion_enable, locomotion_reset;
    // bool action_ctr_enable, action_ctr_reset;

    // define csv
    CSVMaintainer csv(debug_mode);
    csv.init();

    cout << "[MORS Logger]: started" << endl;
    
    double t = 0.0;
    const auto tick_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(dt);
    auto next_tick = now();

    while(true)
    {
        next_tick += tick_period;

        // Put your code here
        // -----------------------------------------------
        
        imu_data = lcmExch.getImuData();
        servo_state = lcmExch.getServoStateData();
        servo_cmd = lcmExch.getServoCmdData();
        // lcmExch.getEnableData(leg_controller_enable, leg_controller_reset,
        //                         locomotion_enable, locomotion_reset,
        //                         action_ctr_enable, action_ctr_reset);
        odometry = lcmExch.getOdometry();
        contact_states = lcmExch.getContactSensorData();
        body_state = lcmExch.getBodyState();
        body_cmd = lcmExch.getRobotCmd();
        leg_state = lcmExch.getLegState();
        body_state_check = lcmExch.getBodyStateCheck();
        leg_state_check = lcmExch.getLegStateCheck();
        if (debug_mode)
        {
            lcmExch.getWbcCmdData(wbc_leg_cmd, wbc_robot_cmd);
            lcmExch.getMpcCmdData(mpc_robot_cmd);
            lcmExch.getPhaseSig(phase, phi);
        }

        // save arrays
        csv.write_servo_state(t, servo_state);
        csv.write_servo_cmd(t, servo_cmd);
        csv.write_imu_data(t, imu_data);
        csv.write_odometry(t, odometry);
        csv.write_contact_states(t, contact_states);
        // csv.write_enable(t, leg_controller_enable, leg_controller_reset,
        //                     locomotion_enable, locomotion_reset,
        //                     action_ctr_enable, action_ctr_reset);
        
        csv.write_robot_state(t, body_state, leg_state);
        csv.write_robot_state_check(t, body_state_check, leg_state_check);
        csv.write_robot_cmd(t, body_cmd);
        if (debug_mode)
        {
            csv.write_mpc_cmd(t, mpc_robot_cmd);
            csv.write_wbc_cmd(t, wbc_leg_cmd, wbc_robot_cmd);
            csv.write_phase_sig(t, phase, phi);
        }
        
        t += module_dt;

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
