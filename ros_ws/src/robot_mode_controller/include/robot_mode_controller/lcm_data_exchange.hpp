#ifndef _lcm_data_exchange_rm_hpp_
#define _lcm_data_exchange_rm_hpp_

#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <lcm/lcm-cpp.hpp>

#include "mors_msgs/servo_cmd_msg.hpp"
#include "mors_msgs/servo_state_msg.hpp"
#include "mors_msgs/enable_msg.hpp"
#include "mors_msgs/robot_state_msg.hpp"
#include "mors_msgs/robot_cmd_msg.hpp"
#include "mors_msgs/phase_signal_msg.hpp"
#include "mors_msgs/wbc_cmd_msg.hpp"
#include "mors_msgs/gait_params_msg.hpp"

// #include "system_functions.hpp"
#include "structs.hpp"
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace Eigen;
using namespace YAML;

string GetEnv( const std::string & var );

class LCMExchanger
{
    public:
        LCMExchanger();
        ~LCMExchanger();

        void start_exchanger();

        void servoStateHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::servo_state_msg* msg);
        void robotStateHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::robot_state_msg* msg);

        void servoStateThread();
        void robotStateThread();

        void getServoStateData(VectorXd &position, VectorXd &velocity, VectorXd &torque);
        RobotData getBodyState();
        LegData getLegState();

        void sendServoCmd(VectorXd &position, VectorXd &velocity, VectorXd &torque, VectorXd &kp, VectorXd &kd);
        void sendEnableData(bool &locomotion_en, bool &locomotion_reset,
                            bool &leg_controller_en, bool &leg_controller_reset);
        void sendWbcCmd(LegData &leg_data, RobotData &robot_data);
        void sendPhaseSig(vector<int>& phase, vector<double>& phi, double t);
        void sendGaitParams(double& t_st, double& t_sw, vector<double>& gait_type, bool& standing, double& stride_height);
        void sendRobotCmd(RobotData& robot_cmd, vector<bool> &active_legs, int8_t& adaptation_type);

    private:
        

        string servo_state_channel, servo_cmd_channel, robot_state_channel;
        string enable_channel, robot_cmd_channel, gait_params_channel;
        string wbc_cmd_channel, phase_signal_channel;

        lcm::LCM servo_state_subscriber;
        lcm::LCM robot_state_subscriber;

        lcm::LCM servo_cmd_publisher;
        lcm::LCM enable_publisher;
        lcm::LCM wbc_cmd_publisher;
        lcm::LCM phase_sig_publisher;
        lcm::LCM robot_cmd_publisher;
        lcm::LCM gait_params_publisher;

        mors_msgs::servo_cmd_msg servoCmdMsg;
        mors_msgs::enable_msg enableMsg;
        mors_msgs::wbc_cmd_msg wbcCmdMsg;
        mors_msgs::phase_signal_msg phaseSigMsg;
        mors_msgs::robot_cmd_msg robotCmdMsg;
        mors_msgs::gait_params_msg gaitPrmsMsg;

        VectorXd servo_pos, servo_vel, servo_torq;

        unique_ptr<thread> thRobotState;
        unique_ptr<thread> thServoState;
        
        bool leg_controller_en, leg_controller_reset;

        // double gear_ratio;
        // bool imp_mode[4];
        // bool grf_mode[4];

        RobotData robot_state;
        RobotData robot_cmd;
        LegData leg_state;

        

};

#endif //_lcm_data_exchange_rm_hpp_