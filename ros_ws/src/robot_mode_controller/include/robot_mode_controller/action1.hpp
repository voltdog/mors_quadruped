#ifndef ACTION1_HPP_
#define ACTION1_HPP_

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <chrono>
#include <thread>
#include <vector>
#include <Eigen/Dense>
#include <unistd.h>

#include "lcm_data_exchange.hpp"
#include "structs.hpp"
#include "trajectory_generator.hpp"


using namespace std::chrono_literals;
using namespace std::chrono;
using namespace std;

// #define JOINTS_KP 2.0
// #define JOINTS_KD 0.4

class GetUp
{
    public:
        // static constexpr double joints_kp = 2.0;//12.0;
        // static constexpr double joints_kd = 0.05;// 0.4;

        GetUp();
        ~GetUp();

        void set_parameters(double dt, double robot_height, double joints_kp, double joints_kd);
        bool is_finished();
        void set_states(const RobotData& robot_state, const LegData& leg_state, const VectorXd& cur_joints_pos);
        void reset();
        void step(bool& action_finished);

    private:
        void joints_kpkd_null();
        void joints_kpkd_inc(double duration);
        void take_joints_position();
        void take_position(std::vector<std::vector<double>> theta_refs);
        void take_velocity(std::vector<std::vector<double>> dtheta_refs);
        void set_blocks_en(bool locomotion_en, bool leg_en);
        int sign(double x);

        LCMExchanger lcm_exch;

        bool finished;

        double dt, freq;
        int dt_ms;
        double robot_height;
        vector<double> leg_kp;
        vector<double> leg_kd;
        
        RobotData robot_state;
        RobotData robot_cmd;//0, robot_cmd_f;
        LegData leg_state;
        VectorXd cur_joints_pos;
        vector<double> robot_height_cmd_traj;
        vector<double> robot_height_vel_cmd_traj;

        VectorXd ref_joints_pos, ref_joints_vel, ref_joints_torq;
        VectorXd joints_kp, joints_kd;

        string start_pos;

        std::vector<std::vector<double>> multiple_ref_joints_pos;
        VectorXd ref_joints_pos_end;

        // bool leg_controller_en;
        bool leg_controller_reset;
        // bool locomotion_en;
        bool locomotion_reset;

        vector<bool> active_legs;
        int8_t adaptation_type;

        double ref_joints_kp;
        double ref_joints_kd;

};

#endif  // ACTION1_HPP_
