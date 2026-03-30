#ifndef _lcm_data_exchange_hpp_
#define _lcm_data_exchange_hpp_

#include <iostream>
#include <atomic>
#include <cstdint>
#include <unistd.h>
#include <chrono>
#include <mutex>
#include <thread>
#include <Eigen/Dense>
#include <lcm/lcm-cpp.hpp>
 
#include "mors_msgs/robot_cmd_msg.hpp"
#include "mors_msgs/mpc_cmd_msg.hpp"
#include "mors_msgs/wbc_cmd_msg.hpp"
#include "mors_msgs/wbc_state_msg.hpp"
#include "mors_msgs/servo_cmd_msg.hpp"
#include "mors_msgs/servo_state_msg.hpp"
#include "mors_msgs/robot_state_msg.hpp"
#include "mors_msgs/gait_params_msg.hpp"
#include "mors_msgs/enable_msg.hpp"
#include "mors_msgs/phase_signal_msg.hpp"
 


#include "system_functions.hpp"
#include <yaml-cpp/yaml.h>
#include "structs.hpp"

using namespace std;
using namespace Eigen;
using namespace YAML;

// #define LEG_CONTROL   1
// #define SERVO_CONTROL 2

class LCMExchanger
{
    public:
        LCMExchanger();
        ~LCMExchanger();

        void start_exchanger();

        void robotCmdHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::robot_cmd_msg* msg);
        
        void robotStateHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::robot_state_msg* msg);
        void servoStateHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::servo_state_msg* msg);

        void gaitParamsHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::gait_params_msg* msg);
        void enableHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::enable_msg* msg);

        RobotData getRobotCmd();
        RobotData getBodyState();
        LegData getLegState();
        ServoStateData getServoState();
        void get_observation_data(RobotData& body_state,
                                    LegData& leg_state,
                                    ServoStateData& servo_state);
        void getWbicObservationStatus(bool& robot_state_received,
                                      bool& servo_state_received) const;
        void get_gait_params(double& t_st, double& t_sw, vector<double>& gait_type, bool& standing, double& stride_height);
        bool get_enable();
        bool get_leg_enable();
        void get_active_legs(vector<bool>& active_legs);
        int get_adaptation_type();
        void get_enable_state(bool& locomotion_enable,
                              bool& leg_controller_enable);

        void sendWbcCmd(RobotData& robot_data, LegData& leg_data);
        void sendPhaseSig(vector<int>& phase, vector<double>& phi, double t);
        void sendMpcCmd(RobotData& robot_cmd, vector<bool> &active_legs);
        void sendServoCmd(const JointVector12d& position,
                          const JointVector12d& velocity,
                          const JointVector12d& torque,
                          const JointVector12d& kp,
                          const JointVector12d& kd);
        void sendWbcState(const JointVector12d& grf);

        bool initialized = false;

    private:
        void robotCmdThread();
        void robotStateThread();
        void servoStateThread();
        void gaitParamsThread();
        void enableThread();

        string robot_cmd_channel, robot_state_channel, servo_state_channel, gait_params_channel, enable_channel;
        string wbc_cmd_channel, wbc_state_channel, phase_signal_channel;
        string mpc_cmd_channel;
        string servo_cmd_channel;

        lcm::LCM robot_cmd_subscriber;
        lcm::LCM robot_state_subscriber;
        lcm::LCM servo_state_subscriber;
        lcm::LCM gait_params_subscriber;
        lcm::LCM enable_subscriber;

        unique_ptr<thread> thRobotCmd;
        unique_ptr<thread> thRobotState;
        unique_ptr<thread> thServoState;
        unique_ptr<thread> thGaitParams;
        unique_ptr<thread> thEnable;

        lcm::LCM wbc_cmd_publisher;
        lcm::LCM wbc_state_publisher;
        lcm::LCM servo_cmd_publisher;
        lcm::LCM phase_sig_publisher;
        lcm::LCM robot_ref_publisher;

        mors_msgs::mpc_cmd_msg mpcCmdMsg;
        mors_msgs::wbc_cmd_msg wbcCmdMsg;
        mors_msgs::wbc_state_msg wbcStateMsg;
        mors_msgs::servo_cmd_msg servoCmdMsg;
        mors_msgs::phase_signal_msg phaseSigMsg;

        RobotData robot_state;
        RobotData robot_cmd;
        LegData leg_state;
        ServoStateData servo_state;
        double t_st;
        double t_sw;
        vector<double> gait_type;
        bool standing;
        double stride_height;
        vector<bool> active_legs;
        int adaptation_type;

        bool locomotion_enable;
        bool leg_controller_enable;
        bool leg_controller_reset;
        std::atomic<bool> robot_state_received_{false};
        std::atomic<bool> servo_state_received_{false};

        mutable std::mutex robot_cmd_mutex;
        mutable std::mutex robot_state_mutex;
        mutable std::mutex servo_state_mutex;
        mutable std::mutex gait_params_mutex;
        mutable std::mutex enable_mutex;
};

#endif //_lcm_data_exchange_hpp_
