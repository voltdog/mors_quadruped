#include <iostream>
#include <exception>
#include <signal.h>
#include <stdlib.h>
#include <csignal>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include "lcm_data_exchange_rm.hpp"
#include "system_functions.hpp"
#include <vbmath.hpp>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;
using namespace YAML;

// current time
auto now() 
{
  return std::chrono::steady_clock::now(); 
}

//Interrupt flag
bool flagLoop = true;
void raiseFlag(int param)
{
    flagLoop = false;
}

int main() {
    cout << "RobotModeController starting..." << endl;
    // load config
    string config_address = mors_sys::GetEnv("CONFIGPATH");
    string emerg_config_address = config_address + "/emergency.yaml";

    YAML::Node emerg_config = YAML::LoadFile(emerg_config_address);
    
    double max_angles[12];
    double min_angles[12];
    for (int i = 0; i < 12; i++)
    {
        max_angles[i] = emerg_config["max_angles"][i].as<double>(); 
        min_angles[i] = emerg_config["min_angles"][i].as<double>(); 
    }

    string dt_config_address = config_address + "/timesteps.yaml";
    YAML::Node dt_config = YAML::LoadFile(dt_config_address);
    double module_dt = dt_config["robot_mode_controller_dt"].as<double>(); 

    auto dt = std::chrono::duration<double>(module_dt);//1ms;
    // auto dt = 5ms;

    LCMExchanger lcmExch;
    lcmExch.start_exchanger();

    bool leg_controller_en = true;
    bool leg_controller_reset = false;
    bool locomotion_en = true;
    bool locomotion_reset = false;

    bool angles_exceeded = false;
    bool emerg_btn_pushed = false;

    VectorXd cur_theta(12);
    VectorXd cur_omega(12);
    VectorXd cur_tau(12);
    VectorXd ref_theta(12);
    VectorXd ref_omega(12);
    VectorXd ref_tau(12);
    VectorXd motor_kp(12);
    VectorXd motor_kd(12);

    ref_theta.setZero(12);
    ref_omega.setZero(12);
    ref_tau.setZero(12);
    motor_kp.setZero(12);
    motor_kd.setZero(12);

    lcmExch.sendServoCmd(ref_theta, ref_omega, ref_tau, motor_kp, motor_kd);

    // 
    int action_num = 0;
    int mode_num = 0;
    bool on_legs = false;

    signal(SIGINT, raiseFlag);

    cout << "RobotModeController started" << endl;

    while(flagLoop == true)
    {
        // Calculating current time
        const auto start{ now() };

        // Put your code here
        // -----------------------------------------------
        
        // read lcm
        lcmExch.getServoStateData(cur_theta, cur_omega, cur_tau);

        // read ros
        // ...

        // check for emergency
        for (int i = 0; i < 12; i++)
        {
            if (cur_theta(i) > max_angles[i] || cur_theta(i) < min_angles[i])
            {
                angles_exceeded = true;
            }
        }


        if (angles_exceeded == false && emerg_btn_pushed == false)
        {
            // enable everything
            leg_controller_en = true;
            locomotion_en = true;
            leg_controller_reset = false;
            locomotion_reset = false;
            
            lcmExch.sendEnableData(locomotion_en, locomotion_reset, 
                                    leg_controller_en, leg_controller_reset);

            // choose working mode
            if (action_num != 0)
                // ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.action()
                cout << "action: " << action_num << endl;
            else
            {
                // action_finished = true
                if (on_legs == true)
                {
                    switch (mode_num)
                    {
                        case 0:
                            cout << "mode: locomotion" << endl;
                            // ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.locomotion_control()
                            break;
                        case 1:
                            cout << "mode: body" << endl;
                            // ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.body_control()
                            break;
                        case 2:
                            cout << "mode: legs" << endl;
                            // ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.ef_control()
                            break;
                        case 3:
                            cout << "mode: joints" << endl;
                            // ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.joint_control()
                            break;
                        default:
                            cout << "mode: do_nothing" << endl;
                            // ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.do_nothing()
                            break;
                    }
                }
                else
                {
                    if (mode_num == 3)
                        // ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.joint_control()
                        cout << "mode: joints" << endl;
                    else
                        // ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.do_nothing()
                        cout << "mode: do_nothing" << endl;
                }
            }
        }
        else
        {
            ref_theta.setZero(12);
            ref_omega.setZero(12);
            ref_tau.setZero(12);
            motor_kp.setZero(12);
            motor_kd.setZero();
            // motor_kd = motor_kd.setOnes(12)*0.6;

            lcmExch.sendServoCmd(ref_theta, ref_omega, ref_tau, motor_kp, motor_kd);

            leg_controller_en = false;
            leg_controller_reset = true;
            locomotion_en = false;
            locomotion_reset = true;

            lcmExch.sendEnableData(locomotion_en, locomotion_reset, 
                                    leg_controller_en, leg_controller_reset);

            cout << "[RobotModeController]: Emergency!" << endl;
            return -1;
        }
        // -----------------------------------------------

        // Wait until spinning time
        while(true)
        {
            std::chrono::duration<double, std::milli> elapsed{now() - start};
            if (elapsed >= dt)
            {
                // cout << "Waited for : " << elapsed.count() << " ms" << endl;
                break;
            }
        }
    }

    std::cout << "[RobotModeController]: Keyboard Interrupt. All programs disabled" << std::endl;

    leg_controller_en = false;
    leg_controller_reset = true;
    locomotion_en = false;
    locomotion_reset = true;

    lcmExch.sendEnableData(locomotion_en, locomotion_reset, 
                            leg_controller_en, leg_controller_reset);
    // return 1;
    ref_theta.setZero(12);
    ref_omega.setZero(12);
    ref_tau.setZero(12);
    motor_kp.setZero(12);
    motor_kd.setZero();
    // motor_kd = motor_kd.setOnes(12)*0.6;

    lcmExch.sendServoCmd(ref_theta, ref_omega, ref_tau, motor_kp, motor_kd);


    return 0;
}