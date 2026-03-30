#ifndef ROBOT_MODE_CONTROLLER_HPP_
#define ROBOT_MODE_CONTROLLER_HPP_

#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <memory>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mors_ros_msgs/msg/gait_params.hpp"
#include "mors_ros_msgs/srv/robot_cmd.hpp"

#include "robot_mode_controller/lcm_data_exchange.hpp"
#include "robot_mode_controller/action1.hpp"
#include "robot_mode_controller/action2.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;
using namespace std::chrono;
using namespace std;

namespace robot_mode_controller
{
    class RobotModeController : public rclcpp::Node
    {
        public:
            RobotModeController(const rclcpp::NodeOptions & options);
            ~RobotModeController();

        private:
            void timer_callback();
            void action();
            void locomotion_control_mode();
            void body_control_mode();
            void standing_mode();

            void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
            void cmd_pose_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
            void gait_params_callback(const mors_ros_msgs::msg::GaitParams::SharedPtr msg);

            void mode_response(const std::shared_ptr<mors_ros_msgs::srv::RobotCmd::Request> request,
                           std::shared_ptr<mors_ros_msgs::srv::RobotCmd::Response> response);

            void action_response(const std::shared_ptr<mors_ros_msgs::srv::RobotCmd::Request> request,
                           std::shared_ptr<mors_ros_msgs::srv::RobotCmd::Response> response);

            void disable_everything();

            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_pose_sub;
            rclcpp::Subscription<mors_ros_msgs::msg::GaitParams>::SharedPtr gait_params_sub;

            rclcpp::Service<mors_ros_msgs::srv::RobotCmd>::SharedPtr mode_service;
            rclcpp::Service<mors_ros_msgs::srv::RobotCmd>::SharedPtr action_service;

            rclcpp::TimerBase::SharedPtr timer_;

            std::float_t Ts;

            LCMExchanger lcmExch;

            bool leg_controller_en;
            bool leg_controller_reset;
            bool locomotion_en;
            bool locomotion_reset;

            bool angles_exceeded;
            bool emerg_btn_pushed;

            VectorXd cur_theta;
            VectorXd cur_omega;
            VectorXd cur_tau;
            VectorXd ref_theta;
            VectorXd ref_omega;
            VectorXd ref_tau;
            VectorXd motor_kp;
            VectorXd motor_kd;

            int action_num;
            int mode_num;
            bool on_legs;

            double max_angles[12];
            double min_angles[12];

            RobotData user_cmd;
            RobotData robot_cmd;
            RobotData robot_state;
            LegData leg_state;

            // gait params
            double t_sw;
            double t_st;
            std::vector<double> phase_offsets;
            bool standing;
            double stride_height;

            // get up action
            GetUp get_up_action;
            bool get_up_started;
            bool action_finished;
            int action_it;

            // lay down action
            LayDown lay_down_action;
            bool lay_down_started;

            bool first_standing;
            vector<bool> active_legs;
            int8_t adaptation_type;

            double robot_base_height;

            double do_nothing_kp;
            double do_nothing_kd;
            double action1_kp;
            double action1_kd;
            double action2_kp;
            double action2_kd;

    };
}

#endif  // ROBOT_MODE_CONTROLLER_HPP_
