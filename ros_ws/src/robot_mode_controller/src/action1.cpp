#include "robot_mode_controller/action1.hpp"

GetUp::GetUp()
{
    finished = false;
    ref_joints_pos.setZero(12);
    ref_joints_vel.setZero(12);
    ref_joints_torq.setZero(12);
    ref_joints_pos_end.setZero(12);

    leg_controller_reset = false;
    locomotion_reset = false;

    active_legs = {true, true, true, true};
    adaptation_type = 0;

    robot_cmd.pos.setZero(3);
    robot_cmd.lin_vel.setZero(3);
    robot_cmd.orientation.setZero(3);
    robot_cmd.ang_vel.setZero(3);
}

void GetUp::set_parameters(double dt, double robot_height, double joints_kp, double joints_kd)
{
    this->dt = dt;
    this->robot_height = robot_height;
    this->ref_joints_kp = joints_kp;
    this->ref_joints_kd = joints_kd;
    freq = 1/dt;
    dt_ms = 1000 * dt;
}

bool GetUp::is_finished()
{
    return finished;
}

void GetUp::set_states(const RobotData& robot_state, const LegData& leg_state, const VectorXd& cur_joints_pos)
{
    this->robot_state = robot_state;
    this->leg_state = leg_state;
    this->cur_joints_pos = cur_joints_pos;
}

void GetUp::reset()
{

}

void GetUp::step(bool& action_finished)
{
    // robot_cmd.orientation(YAW) = robot_state.orientation(YAW);
    // robot_cmd.pos(X) = robot_state.pos(X);
    // robot_cmd.pos(Y) = robot_state.pos(Y);

    set_blocks_en(false, false);

    joints_kpkd_null();

    ref_joints_pos = cur_joints_pos;
    joints_kpkd_inc(0.01);

    if (cur_joints_pos[1] < 0 && cur_joints_pos[4] > 0 && cur_joints_pos[7] < 0 && cur_joints_pos[10] > 0)
    {
        cout << "___" << endl;
        cout << "> >" << endl;
        start_pos = "m";
    }
    else if (cur_joints_pos[1] < 0 && cur_joints_pos[4] > 0 && cur_joints_pos[7] > 0 && cur_joints_pos[10] < 0)
    {
        cout << "___" << endl;
        cout << "> <" << endl;
        start_pos = "x";
    }
    else if (cur_joints_pos[1] > 0 && cur_joints_pos[4] < 0 && cur_joints_pos[7] > 0 && cur_joints_pos[10] < 0)
    {
        cout << "___" << endl;
        cout << "< <" << endl;
        start_pos = "inv_m";
    }
    else if (cur_joints_pos[1] > 0 && cur_joints_pos[4] < 0 && cur_joints_pos[7] < 0 && cur_joints_pos[10] > 0)
    {
        cout << "___" << endl;
        cout << "< >" << endl;
        start_pos = "o";
    }
    else
    {
        start_pos = "z";
        cout << "Strange pose" << endl;
    }

    // go joint to init pose
    // if (start_pos != "x")
    // {
    //     ref_joints_pos_end = cur_joints_pos;
    //     ref_joints_pos_end(0) = -1.57;
    //     ref_joints_pos_end(3) = 1.57;
    //     ref_joints_pos_end(6) = 1.57;
    //     ref_joints_pos_end(9) = -1.57;
    //     ref_joints_pos_end(2) = 3.14;
    //     ref_joints_pos_end(5) = -3.14;
    //     ref_joints_pos_end(8) = -3.14;
    //     ref_joints_pos_end(11) = 3.14;
    //     multiple_ref_joints_pos = traj::create_multiple_trajectory(ref_joints_pos, ref_joints_pos_end, 1.0, dt);
    //     take_position(multiple_ref_joints_pos);

    //     // ref_joints_pos = ref_joints_pos_end;
    //     // ref_joints_pos_end << -1.57, -1.57,  3.14,
    //     //                        1.57,  1.57, -3.14,
    //     //                        1.57,  1.57, -3.14,
    //     //                       -1.57, -1.57,  3.14;
    //     // multiple_ref_joints_pos = traj::create_multiple_trajectory(ref_joints_pos, ref_joints_pos_end, 0.8, dt);
    //     // take_position(multiple_ref_joints_pos);
    //     ref_joints_pos = ref_joints_pos_end;

    //     // std::this_thread::sleep_for(std::chrono::milliseconds(100000));
    // }
    if (start_pos == "x")
    {    
        ref_joints_pos_end << 0.0, -1.57,  3.14,
                            0.0,  1.57, -3.14,
                            0.0,  1.57, -3.14,
                            0.0, -1.57,  3.14;
    }
    else if (start_pos == "m")
    {  
        ref_joints_pos_end << 0.0, -1.57,  3.14,
                            0.0,  1.57, -3.14,
                            0.0, -1.57,  3.14,
                            0.0,  1.57, -3.14;
    }
    else
    {
        ref_joints_pos_end <<   0.0, sign(cur_joints_pos(1))*1.57,  sign(cur_joints_pos(2))*3.14,
                                0.0, sign(cur_joints_pos(4))*1.57,  sign(cur_joints_pos(5))*3.14,
                                0.0, sign(cur_joints_pos(7))*1.57,  sign(cur_joints_pos(8))*3.14,
                                0.0, sign(cur_joints_pos(10))*1.57,  sign(cur_joints_pos(11))*3.14;
    }
    multiple_ref_joints_pos = traj::create_multiple_trajectory(ref_joints_pos, ref_joints_pos_end, 0.5, dt);
    take_position(multiple_ref_joints_pos);

    // touch the ground
    ref_joints_pos = ref_joints_pos_end;
    if (start_pos == "x")
    {
        ref_joints_pos_end << 0.0, -1.4,  2.8,
                            0.0,  1.4, -2.8,
                            0.0,  1.4, -2.8,
                            0.0, -1.4,  2.8;
    }
    else if (start_pos == "m")
    {  
        ref_joints_pos_end << 0.0, -1.4,  2.8,
                            0.0,  1.4, -2.8,
                            0.0, -1.4,  2.8,
                            0.0,  1.4, -2.8;
    }
    else
    {
        ref_joints_pos_end <<   0.0, sign(cur_joints_pos(1))*1.4,  sign(cur_joints_pos(2))*2.8,
                                0.0, sign(cur_joints_pos(4))*1.4,  sign(cur_joints_pos(5))*2.8,
                                0.0, sign(cur_joints_pos(7))*1.4,  sign(cur_joints_pos(8))*2.8,
                                0.0, sign(cur_joints_pos(10))*1.4,  sign(cur_joints_pos(11))*2.8;
    }
    multiple_ref_joints_pos = traj::create_multiple_trajectory(ref_joints_pos, ref_joints_pos_end, 0.5, dt);
    take_position(multiple_ref_joints_pos);


    // start standing by mpc
    bool standing = true;
    double t_st = 0.2; double t_sw = 0.2; vector<double> phase_offsets = {0.0, 0.5, 0.5, 0.0}; double stride_height = 0.06;
    lcm_exch.sendGaitParams(t_st, t_sw, phase_offsets, standing, stride_height);

    cout << "ref height: " << robot_height << endl;
    cout << robot_state.pos.transpose() << endl;
    robot_height_cmd_traj = traj::create_qubic_trajectory(robot_state.pos(Z), robot_height, 1.5, dt);
    robot_height_vel_cmd_traj = traj::create_qubic_vel_trajectory(robot_state.pos(Z), robot_height, 1.5, dt);

    joints_kpkd_null();
    set_blocks_en(true, true);

    for (int i = 0; i < static_cast<int>(robot_height_cmd_traj.size()); i++)
    {
        // cout << robot_height_cmd_traj[i] << endl;
        robot_cmd.pos(Z) = robot_height_cmd_traj[i];
        robot_cmd.lin_vel(Z) = robot_height_vel_cmd_traj[i];
        lcm_exch.sendRobotCmd(robot_cmd, active_legs, adaptation_type);
        std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
    }

    action_finished = true;
}

void GetUp::joints_kpkd_null()
{
    joints_kp.setZero(12);
    joints_kd.setZero(12);
    lcm_exch.sendServoCmd(ref_joints_pos, ref_joints_vel, ref_joints_torq, joints_kp, joints_kd);
}

void GetUp::set_blocks_en(bool locomotion_en, bool leg_en)
{
    lcm_exch.sendEnableData(locomotion_en, locomotion_reset, 
                            leg_en, leg_controller_reset);
}

void GetUp::joints_kpkd_inc(double duration)
{
    double ts = freq * duration;
    double kp_inc = ref_joints_kp / ts;
    double kd_inc = ref_joints_kd / ts;
    
    // cout << dt_ms << endl;

    for (int k = 0; k < int(ts); k++)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int i = 0; i < 3; i++)
            {
                if (joints_kp(i) < ref_joints_kp)
                    joints_kp(i + 3*j) += kp_inc;

                if (joints_kd(i) < ref_joints_kd)
                    joints_kd(i + 3*j) += kd_inc;
            }
            lcm_exch.sendServoCmd(ref_joints_pos, ref_joints_vel, ref_joints_torq, joints_kp, joints_kd);
            std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
        }
    }
}

void GetUp::take_position(std::vector<std::vector<double>> theta_refs)
{
    // VectorXd theta_ref(12);
    // for (long unsigned int i = 0; i < theta_refs.size(); i++)
    // {
    //     ref_joints_pos = theta_refs[i];
        
    // }

    VectorXd ref_pos_(12);
    for (long unsigned int i = 0; i < theta_refs[0].size(); i++)
    {
        for (int j = 0; j < 12; j++)
        {
            ref_pos_(j) = theta_refs[j][i];
            
        }
        // cout << ref_pos_(3) << endl;
        lcm_exch.sendServoCmd(ref_pos_, ref_joints_vel, ref_joints_torq, joints_kp, joints_kd);
        std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
    }
}

int GetUp::sign(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

GetUp::~GetUp()
{

}