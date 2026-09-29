#ifndef ACTION2_HPP_
#define ACTION2_HPP_

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

// #define JOINTS_KP 12.0
// #define JOINTS_KD 0.4

class LayDown
{
    public:
        // static constexpr double JOINTS_KP = 2.0;//12.0;
        // static constexpr double JOINTS_KD = 0.05;// 0.4;

        LayDown();
        ~LayDown();

        void set_parameters(double dt, double robot_height, double joints_kp, double joints_kd);
        bool is_finished();
        void set_states(RobotData& robot_state, LegData& leg_state, VectorXd& cur_joints_pos);
        void reset();
        void step(bool& action_finished);

    private:
        void joints_kpkd_null();
        void joints_kpkd_max();
        void joints_kpkd_set(double kp, double kd);
        void joints_kpkd_inc(double duration);
        void take_joints_position();
        void take_position(std::vector<std::vector<double>> theta_refs);
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

#endif  // ACTION2_HPP_
