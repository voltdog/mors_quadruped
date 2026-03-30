#include "robot_mode_controller/robot_mode_controller.hpp"

namespace robot_mode_controller
{
    RobotModeController::RobotModeController(const rclcpp::NodeOptions & options)
        : Node("robot_mode_controller", options)
    {
        // read config
        string config_address = GetEnv("CONFIGPATH");
        string emerg_config_address = config_address + "/emergency.yaml";
        YAML::Node emerg_config = YAML::LoadFile(emerg_config_address);
        
        for (int i = 0; i < 12; i++)
        {
            max_angles[i] = emerg_config["max_angles"][i].as<double>(); 
            min_angles[i] = emerg_config["min_angles"][i].as<double>(); 
        }

        // string stance_config_address = config_address + "/stance_controller_mpc.yaml";
        // YAML::Node stance_config = YAML::LoadFile(stance_config_address);
        // robot_base_height = stance_config["robot_height"].as<double>(); 

        string action_config_address = config_address + "/action_controller.yaml";
        YAML::Node action_config = YAML::LoadFile(action_config_address);
        do_nothing_kd = action_config["do_nothing_kd"].as<double>();
        do_nothing_kp = action_config["do_nothing_kp"].as<double>();
        action1_kd = action_config["action1_kd"].as<double>();
        action1_kp = action_config["action1_kp"].as<double>();
        action2_kd = action_config["action2_kd"].as<double>();
        action2_kp = action_config["action2_kp"].as<double>();

        // create subscribers
        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&RobotModeController::cmd_vel_callback, this, _1));
        cmd_pose_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_pose", 10, std::bind(&RobotModeController::cmd_pose_callback, this, _1));
        gait_params_sub = this->create_subscription<mors_ros_msgs::msg::GaitParams>("gait_params", 10, std::bind(&RobotModeController::gait_params_callback, this, _1));

        // create services
        mode_service = this->create_service<mors_ros_msgs::srv::RobotCmd>("robot_mode", std::bind(&RobotModeController::mode_response, this, _1, _2));
        action_service = this->create_service<mors_ros_msgs::srv::RobotCmd>("robot_action", std::bind(&RobotModeController::action_response, this, _1, _2));

        // init timer callback
        Ts = 0.002;
        auto timer_period = Ts * 1000ms;
        timer_ = this->create_wall_timer(timer_period, std::bind(&RobotModeController::timer_callback, this));

        // init lcm
        lcmExch.start_exchanger();

        // define variables
        leg_controller_en = true;
        leg_controller_reset = false;
        locomotion_en = true;
        locomotion_reset = false;

        angles_exceeded = false;
        emerg_btn_pushed = false;

        cur_theta.setZero(12);
        cur_omega.setZero(12);
        cur_tau.setZero(12);
        ref_theta.setZero(12);
        ref_omega.setZero(12);
        ref_tau.setZero(12);
        motor_kp.setZero(12);
        motor_kd.setZero(12);

        lcmExch.sendServoCmd(ref_theta, ref_omega, ref_tau, motor_kp, motor_kd);

        // 
        action_num = 0;
        mode_num = 0;
        on_legs = false;

        t_sw = 0.2;
        t_st = 0.3;
        phase_offsets = {0.0, 0.0, 0.0, 0.0};
        stride_height = 0.06;

        user_cmd.lin_vel.setZero(3);
        user_cmd.ang_vel.setZero(3);
        user_cmd.pos.setZero(3);
        user_cmd.orientation.setZero(3);
        robot_cmd.lin_vel.setZero(3);
        robot_cmd.ang_vel.setZero(3);
        robot_cmd.pos.setZero(3);
        robot_cmd.orientation.setZero(3);
        robot_state.lin_vel.setZero(3);
        robot_state.ang_vel.setZero(3);
        robot_state.pos.setZero(3);
        robot_state.orientation.setZero(3);
        leg_state.contacts = {false, false, false, false};
        leg_state.r1_pos.setZero(3);
        leg_state.l1_pos.setZero(3);
        leg_state.r2_pos.setZero(3);
        leg_state.l2_pos.setZero(3);
        leg_state.r1_vel.setZero(3);
        leg_state.l1_vel.setZero(3);
        leg_state.r2_vel.setZero(3);
        leg_state.l2_vel.setZero(3);

        first_standing = true;

        active_legs = {true, true, true, true};
        adaptation_type = 0;
    
        // actions
        get_up_action.set_parameters(Ts, 0.2, action1_kp, action1_kd);
        get_up_started = false;
        action_finished = false;
        action_it = 0;

        lay_down_action.set_parameters(Ts, 0.2, action2_kp, action2_kd); 
        lay_down_started = false;

    }

    void RobotModeController::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Cmd Vel Received: '%f'", msg->linear.x);
        user_cmd.lin_vel(X) = msg->linear.x;
        user_cmd.lin_vel(Y) = msg->linear.y;
        user_cmd.lin_vel(Z) = msg->linear.z;

        user_cmd.ang_vel(X) = msg->angular.x;
        user_cmd.ang_vel(Y) = msg->angular.y;
        user_cmd.ang_vel(Z) = msg->angular.z;

    }

    void RobotModeController::cmd_pose_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Cmd Pose Received: '%f'", msg->linear.x);
        user_cmd.pos(X) = msg->linear.x;
        user_cmd.pos(Y) = msg->linear.y;
        user_cmd.pos(Z) = msg->linear.z; //+ robot_base_height;
        // RCLCPP_INFO(this->get_logger(), "Cmd Pose Received Z: '%f'", msg->linear.z);

        user_cmd.orientation(X) = msg->angular.x;
        user_cmd.orientation(Y) = msg->angular.y;
        user_cmd.orientation(Z) = msg->angular.z;
    }

    void RobotModeController::gait_params_callback(const mors_ros_msgs::msg::GaitParams::SharedPtr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Cmd Pose Received: '%f'", msg->t_sw);
        t_st = msg->t_st;
        t_sw = msg->t_sw;
        standing = msg->standing;
        stride_height = msg->stride_height;
        for(int i = 0; i < 4; i++)
            phase_offsets[i] = msg->gait_offsets[i];
    }

    void RobotModeController::mode_response(const std::shared_ptr<mors_ros_msgs::srv::RobotCmd::Request> request,
                           std::shared_ptr<mors_ros_msgs::srv::RobotCmd::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Mode Request: '%d'", request->data);
        mode_num = request->data;
        response->success = 1;
    }

    void RobotModeController::action_response(const std::shared_ptr<mors_ros_msgs::srv::RobotCmd::Request> request,
                    std::shared_ptr<mors_ros_msgs::srv::RobotCmd::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Action Request: '%d'", request->data);
        action_num = request->data;
        response->success = 1;
    }

    void RobotModeController::timer_callback()
    {
        // RCLCPP_INFO(this->get_logger(), "hello");
        // read lcm
        lcmExch.getServoStateData(cur_theta, cur_omega, cur_tau);
        robot_state = lcmExch.getBodyState();
        leg_state = lcmExch.getLegState();


        // cout << robot_state.ang_vel.transpose() << endl;
        // cout << robot_state.lin_vel.transpose() << endl;
        // cout << robot_state.orientation.transpose() << endl;
        // cout << robot_state.pos.transpose() << endl;
        // cout << "===" << endl;
        // cout << leg_state.contacts[0] << " " << leg_state.contacts[1] << " " << leg_state.contacts[2] << " " << leg_state.contacts[3] << endl;
        // cout << leg_state.r1_pos.transpose() << endl;
        // cout << leg_state.l1_pos.transpose() << endl;
        // cout << leg_state.r2_pos.transpose() << endl;
        // cout << leg_state.l2_pos.transpose() << endl;
        

        if (emerg_btn_pushed == false)
        {
            // choose working mode
            if (action_num != 0)
            {
                // ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.action()
                // cout << "action: " << action_num << endl;
                action();
            }
            else
            {
                
                // action_finished = true
                // angles_exceeded = false;
                // on_legs = true;
                if (angles_exceeded == false && on_legs == true)
                {
                    // check for button emergency
                    for (int i = 0; i < 12; i++)
                    {
                        if (cur_theta(i) > max_angles[i] || cur_theta(i) < min_angles[i])
                        {
                            angles_exceeded = true;
                            cout << "Joint angle exeeded! Joint Num: " << i << " Angle Value: " << cur_theta(i) << endl; 
                        }
                        // else
                        //     angles_exceeded = false;
                    }
                    

                    // enable everything
                    leg_controller_en = true;
                    locomotion_en = true;
                    leg_controller_reset = false;
                    locomotion_reset = false;
                    
                    lcmExch.sendEnableData(locomotion_en, locomotion_reset, 
                                            leg_controller_en, leg_controller_reset);
                    switch (mode_num)
                    {
                        case 1:
                            // cout << "mode: locomotion" << endl;
                            locomotion_control_mode();
                            // ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.locomotion_control()
                            break;
                        case 2:
                            // cout << "mode: body" << endl;
                            body_control_mode();
                            // ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.body_control()
                            break;
                        case 3:
                            // cout << "mode: standing" << endl;
                            standing_mode();
                            // ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.ef_control()
                            break;
                        case 4:
                            cout << "mode: legs" << endl;
                            // ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.joint_control()
                            break;
                        default:
                            // cout << "mode: do_nothing" << endl;
                            // ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.do_nothing()
                            break;
                    }
                }
                else
                {
                    // disable everything
                    disable_everything();
                    // cout << "emergency!" << endl;

                    // if (mode_num == 3)
                    //     // ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.joint_control()
                    //     cout << "mode: joints" << endl;
                    // else
                        // ref_servo_pos, ref_servo_vel, ref_servo_torq, ref_servo_kp, ref_servo_kd = self.do_nothing()
                        // cout << "mode: do_nothing laying down" << endl;
                }
            }
        }
        else
        {
            // stop everything if emergency
            disable_everything();

            cout << "[RobotModeController]: Emergency!" << endl;
            return;
        }
        if (mode_num != 3)
            first_standing = true;
    }

    void RobotModeController::locomotion_control_mode()
    {
        // cout << standing << endl;
        // if ((abs(user_cmd.pos(X) - robot_state.pos(X)) < 0.01) && (abs(user_cmd.lin_vel(X)) <= 0.05) )
        // if (abs(user_cmd.lin_vel(X)) <= 0.05) 
        // {
        //     standing = true;
        // }
        // else
        // {
        //     standing = false;
        // }

        robot_cmd = user_cmd;
        lcmExch.sendRobotCmd(robot_cmd, active_legs, adaptation_type);
        lcmExch.sendGaitParams(t_st, t_sw, phase_offsets, standing, stride_height);
    }

    void RobotModeController::body_control_mode()
    {
        robot_cmd = user_cmd;
        standing = true;
        lcmExch.sendRobotCmd(robot_cmd, active_legs, adaptation_type);
        lcmExch.sendGaitParams(t_st, t_sw, phase_offsets, standing, stride_height);
    }

    void RobotModeController::standing_mode()
    {
        standing = true;
        if (first_standing == true)
        {
            robot_cmd = user_cmd;
            first_standing = false;
        }
        robot_cmd.pos(Z) = user_cmd.pos(Z);
        lcmExch.sendRobotCmd(robot_cmd, active_legs, adaptation_type);
        lcmExch.sendGaitParams(t_st, t_sw, phase_offsets, standing, stride_height);
    }

    void RobotModeController::action()
    {
        switch (action_num)
        {
            case 1: // stand up
                cout << "getting up to Z pos: " << user_cmd.pos(Z) << endl;
                get_up_action.set_parameters(Ts, user_cmd.pos(Z), action1_kp, action1_kd);
                get_up_action.reset();
                cout << "pos_x: " << robot_state.pos(X) << endl;
                get_up_action.set_states(robot_state, leg_state, cur_theta);
                get_up_action.step(action_finished);
                action_num = 0;
                // robot_cmd.orientation(YAW) = robot_state.orientation(YAW);
                // robot_cmd.pos(X) = robot_state.pos(X);
                // robot_cmd.pos(Y) = robot_state.pos(Y);
                angles_exceeded = false;
                on_legs = true;
                break;
            case 2: // lay down
                cout << "laying down" << endl;
                if (angles_exceeded == false)
                {
                    lay_down_action.set_parameters(Ts, user_cmd.pos(Z), action2_kp, action2_kd);
                    lay_down_action.reset();
                    lay_down_action.set_states(robot_state, leg_state, cur_theta);
                    lay_down_action.step(action_finished);
                }
                action_num = 0;
                on_legs = false;
                break;
            case 3: // give paw
                //
                break;
            default:
                action_num = 0;
                break;
        }
    }
    
    RobotModeController::~RobotModeController()
    {
        std::cout << "[RobotModeController]: Keyboard Interrupt or unrecognized error. All programs disabled" << std::endl;

        // leg_controller_en = false;
        // leg_controller_reset = true;
        // locomotion_en = false;
        // locomotion_reset = true;

        // lcmExch.sendEnableData(locomotion_en, locomotion_reset, 
        //                         leg_controller_en, leg_controller_reset);
        // // return 1;
        // ref_theta.setZero(12);
        // ref_omega.setZero(12);
        // ref_tau.setZero(12);
        // motor_kp.setZero(12);
        // motor_kd.setZero(12);
        // // motor_kd = motor_kd.setOnes(12)*0.6;

        // lcmExch.sendServoCmd(ref_theta, ref_omega, ref_tau, motor_kp, motor_kd);
        disable_everything();
    }

    void RobotModeController::disable_everything()
    {
        leg_controller_en = false;
        leg_controller_reset = true;
        locomotion_en = false;
        locomotion_reset = true;

        ref_theta.setZero(12);
        ref_omega.setZero(12);
        ref_tau.setZero(12);
        motor_kp.setZero(12);
        // motor_kd.setZero(12);
        motor_kd = motor_kd.setOnes(12)*do_nothing_kd;//*0.6;

        lcmExch.sendEnableData(locomotion_en, locomotion_reset, 
                                leg_controller_en, leg_controller_reset);
        lcmExch.sendServoCmd(ref_theta, ref_omega, ref_tau, motor_kp, motor_kd);
    }
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<robot_mode_controller::RobotModeController>(options));
    rclcpp::shutdown();
    return 0;
}