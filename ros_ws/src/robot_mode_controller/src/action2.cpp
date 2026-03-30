#include "robot_mode_controller/action2.hpp"

LayDown::LayDown()
{
    finished = false;
    ref_joints_pos.setZero(12);
    ref_joints_vel.setZero(12);
    ref_joints_torq.setZero(12);
    ref_joints_pos_end.setZero(12);
    joints_kp.setZero(12);
    joints_kd.setZero(12);

    leg_controller_reset = false;
    locomotion_reset = false;

    active_legs = {true, true, true, true};
    adaptation_type = 0;

    robot_cmd.pos.setZero(3);
    robot_cmd.lin_vel.setZero(3);
    robot_cmd.orientation.setZero(3);
    robot_cmd.ang_vel.setZero(3);

}

void LayDown::set_parameters(double dt, double robot_height, double joints_kp, double joints_kd)
{
    this->dt = dt;
    this->robot_height = robot_height;
    this->ref_joints_kp = joints_kp;
    this->ref_joints_kd = joints_kd;
    freq = 1/dt;
    dt_ms = 1000 * dt;
}

bool LayDown::is_finished()
{
    return finished;
}

void LayDown::set_states(RobotData& robot_state, LegData& leg_state, VectorXd& cur_joints_pos)
{
    this->robot_state = robot_state;
    this->leg_state = leg_state;
    this->cur_joints_pos = cur_joints_pos;
}

void LayDown::reset()
{

}

void LayDown::step(bool& action_finished)
{
    // start laying down by mpc
    // set_blocks_en(true, true);
    // robot_height_cmd_traj = traj::create_qubic_trajectory(robot_state.pos(Z), robot_state.pos(Z)/1.5, 1.0, dt);
    
    // for (int i = 0; i < static_cast<int>(robot_height_cmd_traj.size()); i++)
    // {
    //     cout << robot_height_cmd_traj[i] << endl;
    //     robot_cmd.pos(Z) = robot_height_cmd_traj[i];
    //     lcm_exch.sendRobotCmd(robot_cmd, active_legs, adaptation_type);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
    // }

    

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

    ref_joints_pos = cur_joints_pos;
    
    joints_kpkd_set(ref_joints_kp, ref_joints_kd);
    set_blocks_en(false, false);

    // got to the ground
    if (start_pos == "x")
    {  
        ref_joints_pos_end <<   cur_joints_pos(0), -1.57,  3.14,
                                cur_joints_pos(3),  1.57, -3.14,
                                cur_joints_pos(6),  1.57, -3.14,
                                cur_joints_pos(9), -1.57,  3.14;
    }
    else if (start_pos == "m")
    {  
        ref_joints_pos_end <<   cur_joints_pos(0), -1.57,  3.14,
                                cur_joints_pos(3),  1.57, -3.14,
                                cur_joints_pos(6), -1.57,  3.14,
                                cur_joints_pos(9),  1.57, -3.14;
    }
    else
    {
        ref_joints_pos_end <<   cur_joints_pos(0), sign(cur_joints_pos(1))*1.57,  sign(cur_joints_pos(2))*3.14,
                                cur_joints_pos(3), sign(cur_joints_pos(4))*1.57,  sign(cur_joints_pos(5))*3.14,
                                cur_joints_pos(6), sign(cur_joints_pos(7))*1.57,  sign(cur_joints_pos(8))*3.14,
                                cur_joints_pos(9), sign(cur_joints_pos(10))*1.57,  sign(cur_joints_pos(11))*3.14;
    }
    multiple_ref_joints_pos = traj::create_multiple_trajectory(ref_joints_pos, ref_joints_pos_end, 2.0, dt);
    
    take_position(multiple_ref_joints_pos);

    // go shoulder to null
    ref_joints_pos = ref_joints_pos_end;
    if (start_pos == "x")
    {  
        ref_joints_pos_end <<   0.0, -1.57,  3.14,
                                0.0,  1.57, -3.14,
                                0.0,  1.57, -3.14,
                                0.0, -1.57,  3.14;
    }
    else if (start_pos == "m")
    {  
        ref_joints_pos_end <<   0.0, -1.57,  3.14,
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

    joints_kpkd_set(0.0, ref_joints_kd*2);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    joints_kpkd_null();

    action_finished = true;
}

void LayDown::joints_kpkd_null()
{
    joints_kp.setZero(12);
    joints_kd.setZero(12);
    lcm_exch.sendServoCmd(ref_joints_pos, ref_joints_vel, ref_joints_torq, joints_kp, joints_kd);
}

void LayDown::joints_kpkd_set(double kp, double kd)
{
    joints_kp.setConstant(kp);
    joints_kd.setConstant(kd);
    lcm_exch.sendServoCmd(ref_joints_pos, ref_joints_vel, ref_joints_torq, joints_kp, joints_kd);
}

void LayDown::joints_kpkd_max()
{
    for (int i = 0; i < 12; i++)
    {
        joints_kp(i) = ref_joints_kp;
        joints_kd(i) = ref_joints_kd;
    }
    
    lcm_exch.sendServoCmd(ref_joints_pos, ref_joints_vel, ref_joints_torq, joints_kp, joints_kd);
}

void LayDown::set_blocks_en(bool locomotion_en, bool leg_en)
{
    lcm_exch.sendEnableData(locomotion_en, locomotion_reset, 
                            leg_en, leg_controller_reset);
}

void LayDown::joints_kpkd_inc(double duration)
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

void LayDown::take_position(std::vector<std::vector<double>> theta_refs)
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

int LayDown::sign(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

LayDown::~LayDown()
{

}