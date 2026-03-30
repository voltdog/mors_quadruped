#include "csv_maintaner.hpp"

CSVMaintainer::CSVMaintainer(bool debug_mode)
    : debug_mode(debug_mode)
{

}

CSVMaintainer::~CSVMaintainer()
{

}

void CSVMaintainer::init()
{
    // creat new folder with datetime postfix
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%y%m%d_%H%M%S");//"%d-%m-%Y %H-%M-%S");
    auto folder_postfix = oss.str();
    string user = mors_sys::GetEnv("USER");
    log_folder = "/home/" + user + "/mors_logs/log_" + folder_postfix + "/";
    filesystem::create_directories(log_folder);
    cout << "[MORS Logger]: folder addr: " << log_folder << endl;

    // define log file addresses
    robot_cmd_addr = log_folder + "robot_cmd.csv";
    mpc_cmd_addr = log_folder + "mpc_cmd.csv";
    wbc_cmd_addr = log_folder + "wbc_cmd.csv";
    servo_cmd_addr = log_folder + "servo_cmd.csv";

    robot_state_addr = log_folder + "robot_state.csv";
    robot_state_check_addr = log_folder + "robot_state_check.csv";
    servo_state_addr = log_folder + "servo_state.csv";

    imu_data_addr = log_folder + "imu_data.csv";
    odometry_addr = log_folder + "odometry.csv";
    phase_sig_addr = log_folder + "gait_phase.csv";
    contact_sensor_addr = log_folder + "contact_sensor.csv";

    enable_addr = log_folder + "enable.csv";

    // define table head
    const vector<string> servo_state_head = {"time", 
        "pos/0", "pos/1", "pos/2", "pos/3", "pos/4", "pos/5", "pos/6", "pos/7", "pos/8", "pos/9", "pos/10", "pos/11",
        "vel/0", "vel/1", "vel/2", "vel/3", "vel/4", "vel/5", "vel/6", "vel/7", "vel/8", "vel/9", "vel/10", "vel/11",
        "torq/0", "torq/1", "torq/2", "torq/3", "torq/4", "torq/5", "torq/6", "torq/7", "torq/8", "torq/9", "torq/10", "torq/11"};
    
    const vector<string> servo_state_filt_head = {"time", 
        "pos/0", "pos/1", "pos/2", "pos/3", "pos/4", "pos/5", "pos/6", "pos/7", "pos/8", "pos/9", "pos/10", "pos/11",
        "vel/0", "vel/1", "vel/2", "vel/3", "vel/4", "vel/5", "vel/6", "vel/7", "vel/8", "vel/9", "vel/10", "vel/11",
        "torq/0", "torq/1", "torq/2", "torq/3", "torq/4", "torq/5", "torq/6", "torq/7", "torq/8", "torq/9", "torq/10", "torq/11"};

    const vector<string> servo_cmd_head = {"time", 
        "pos/0", "pos/1", "pos/2", "pos/3", "pos/4", "pos/5", "pos/6", "pos/7", "pos/8", "pos/9", "pos/10", "pos/11",
        "vel/0", "vel/1", "vel/2", "vel/3", "vel/4", "vel/5", "vel/6", "vel/7", "vel/8", "vel/9", "vel/10", "vel/11",
        "torq/0", "torq/1", "torq/2", "torq/3", "torq/4", "torq/5", "torq/6", "torq/7", "torq/8", "torq/9", "torq/10", "torq/11",
        "kp/0", "kp/1", "kp/2", "kp/3", "kp/4", "kp/5", "kp/6", "kp/7", "kp/8", "kp/9", "kp/10", "kp/11",
        "kd/0", "kd/1", "kd/2", "kd/3", "kd/4", "kd/5", "kd/6", "kd/7", "kd/8", "kd/9", "kd/10", "kd/11"};
    
    const vector<string> contact_sensor_head = {"time", 
        "r1", "l1", "r2", "l2"};

    const vector<string> imu_data_head = {"time", 
        "orientation_euler/0", "orientation_euler/1", "orientation_euler/2", 
        "orientation_quaternion/0", "orientation_quaternion/1", "orientation_quaternion/2", "orientation_quaternion/3", 
        "ang_vel/0", "ang_vel/1", "ang_vel/2", 
        "lin_accel/0", "lin_accel/1", "lin_accel/2"};

    const vector<string> wbc_cmd_head = {"time", 
        "body_pos/0", "body_pos/1", "body_pos/2",
        "body_euler/0", "body_euler/1", "body_euler/2",
        "body_lin_vel/0", "body_lin_vel/1", "body_lin_vel/2",
        "body_ang_vel/0", "body_ang_vel/1", "body_ang_vel/2",
        "r1_pos/0", "r1_pos/1", "r1_pos/2", 
        "l1_pos/0", "l1_pos/1", "l1_pos/2", 
        "r2_pos/0", "r2_pos/1", "r2_pos/2", 
        "l2_pos/0", "l2_pos/1", "l2_pos/2", 
        "r1_vel/0", "r1_vel/1", "r1_vel/2", 
        "l1_vel/0", "l1_vel/1", "l1_vel/2", 
        "r2_vel/0", "r2_vel/1", "r2_vel/2", 
        "l2_vel/0", "l2_vel/1", "l2_vel/2", 
        "r1_grf/0", "r1_grf/1", "r1_grf/2", 
        "l1_grf/0", "l1_grf/1", "l1_grf/2", 
        "r2_grf/0", "r2_grf/1", "r2_grf/2", 
        "l2_grf/0", "l2_grf/1", "l2_grf/2"};

    const vector<string> mpc_cmd_head = {"time", 
        "body_pos/0", "body_pos/1", "body_pos/2",
        "body_euler/0", "body_euler/1", "body_euler/2",
        "body_lin_vel/0", "body_lin_vel/1", "body_lin_vel/2",
        "body_ang_vel/0", "body_ang_vel/1", "body_ang_vel/2"};

    const vector<string> enable_head = {"time", 
        "leg_control_en", "leg_control_reset",
        "locomotion_en", "locomotion_reset",
        "action_ctr_en", "action_ctr_reset"};

    const vector<string> odometry_head = {"time", 
        "pos/0", "pos/1", "pos/2",
        "rpy/0", "rpy/1", "rpy/2",
        "lin_vel/0", "lin_vel/1", "lin_vel/2",
        "ang_vel/0", "ang_vel/1", "ang_vel/2"};

    const vector<string> robot_state_head = {"time", 
        "pos/0", "pos/1", "pos/2",
        "rpy/0", "rpy/1", "rpy/2",
        "lin_vel/0", "lin_vel/1", "lin_vel/2",
        "ang_vel/0", "ang_vel/1", "ang_vel/2",
        "r1_pos/0", "r1_pos/1", "r1_pos/2", 
        "l1_pos/0", "l1_pos/1", "l1_pos/2", 
        "r2_pos/0", "r2_pos/1", "r2_pos/2", 
        "l2_pos/0", "l2_pos/1", "l2_pos/2", 
        "r1_vel/0", "r1_vel/1", "r1_vel/2", 
        "l1_vel/0", "l1_vel/1", "l1_vel/2", 
        "r2_vel/0", "r2_vel/1", "r2_vel/2", 
        "l2_vel/0", "l2_vel/1", "l2_vel/2", 
        "r1_grf/0", "r1_grf/1", "r1_grf/2", 
        "l1_grf/0", "l1_grf/1", "l1_grf/2", 
        "r2_grf/0", "r2_grf/1", "r2_grf/2", 
        "l2_grf/0", "l2_grf/1", "l2_grf/2",
        "contact/0", "contact/1", "contact/2", "contact/3"};

    const vector<string> robot_state_check_head = robot_state_head;

    const vector<string> robot_cmd_head = {"time", 
        "pos/0", "pos/1", "pos/2",
        "rpy/0", "rpy/1", "rpy/2",
        "lin_vel/0", "lin_vel/1", "lin_vel/2",
        "ang_vel/0", "ang_vel/1", "ang_vel/2"};

    const vector<string> phase_sig_head = {"time", 
        "phase/0", "phase/1", "phase/2", "phase/3",
        "phi/0", "phi/1", "phi/2", "phi/3"};
   
    // write headers to files
    create_csv(servo_state_csv, servo_state_head, servo_state_addr);
    // create_csv(servo_state_filt_csv, servo_state_filt_head, servo_state_filt_addr);
    create_csv(servo_cmd_csv, servo_cmd_head, servo_cmd_addr);
    create_csv(imu_data_csv, imu_data_head, imu_data_addr);
    create_csv(contact_sensor_csv, contact_sensor_head, contact_sensor_addr);
    if (debug_mode)
    {
        create_csv(mpc_cmd_csv, mpc_cmd_head, mpc_cmd_addr);
        create_csv(wbc_cmd_csv, wbc_cmd_head, wbc_cmd_addr);
    }
    // create_csv(enable_csv, enable_head, enable_addr);
    create_csv(odometry_csv, odometry_head, odometry_addr);
    create_csv(robot_state_csv, robot_state_head, robot_state_addr);
    create_csv(robot_state_check_csv, robot_state_check_head, robot_state_check_addr);
    create_csv(robot_cmd_csv, robot_cmd_head, robot_cmd_addr);
    if (debug_mode)
    {
        create_csv(phase_sig_csv, phase_sig_head, phase_sig_addr);
    }
}

void CSVMaintainer::create_csv(CSVWriter &csv, const vector<string> &head, string &addr)
{
    csv.set_separator(", ");
    for (string head : head)
        csv << head;    
    csv.writeToFile(addr);
}

void CSVMaintainer::write_contact_states(double t, vector<bool>& contact_states)
{
    contact_sensor_csv.resetContent();
    contact_sensor_csv << t;

    for (int i = 0; i < 4; i++)
        contact_sensor_csv << contact_states[i];

    contact_sensor_csv.writeToFile(contact_sensor_addr, true);
}

void CSVMaintainer::write_servo_state(double time, ServoData &servo_state)
{
    servo_state_csv.resetContent();
    servo_state_csv << time;

    for (int i=0; i<12; i++) {
        if (std::abs(servo_state.pos(i)) < 1e-4) 
            servo_state_csv << 0.0;
        else
            servo_state_csv << servo_state.pos(i);
    }
        
    for (int i=0; i<12; i++) {
        if (std::abs(servo_state.vel(i)) < 1e-4)
            servo_state_csv << 0.0;
        else
            servo_state_csv << servo_state.vel(i);
    }
        
    for (int i=0; i<12; i++) {
        if (std::abs(servo_state.torq(i)) < 1e-4)
            servo_state_csv << 0.0;
        else
            servo_state_csv << servo_state.torq(i);
    }

    servo_state_csv.writeToFile(servo_state_addr, true);

}

void CSVMaintainer::write_servo_state_filt(double time, ServoData &servo_state_filt)
{
    servo_state_filt_csv.resetContent();
    servo_state_filt_csv << time;

    for (int i=0; i<12; i++) {
        if (std::abs(servo_state_filt.pos(i)) < 1e-4) 
            servo_state_filt_csv << 0.0;
        else
            servo_state_filt_csv << servo_state_filt.pos(i);
    }
        
    for (int i=0; i<12; i++) {
        if (std::abs(servo_state_filt.vel(i)) < 1e-4)
            servo_state_filt_csv << 0.0;
        else
            servo_state_filt_csv << servo_state_filt.vel(i);
    }
        
    for (int i=0; i<12; i++) {
        if (std::abs(servo_state_filt.torq(i)) < 1e-4)
            servo_state_filt_csv << 0.0;
        else
            servo_state_filt_csv << servo_state_filt.torq(i);
    }
        

    servo_state_filt_csv.writeToFile(servo_state_filt_addr, true);
}

void CSVMaintainer::write_servo_cmd(double time, ServoData &servo_cmd)
{
    servo_cmd_csv.resetContent();
    servo_cmd_csv << time;

    for (int i=0; i<12; i++)
        servo_cmd_csv << servo_cmd.pos(i);
    for (int i=0; i<12; i++)
        servo_cmd_csv << servo_cmd.vel(i);
    for (int i=0; i<12; i++)
        servo_cmd_csv << servo_cmd.torq(i);
    for (int i=0; i<12; i++)
        servo_cmd_csv << servo_cmd.kp(i);
    for (int i=0; i<12; i++)
        servo_cmd_csv << servo_cmd.kd(i);

    servo_cmd_csv.writeToFile(servo_cmd_addr, true);

}

void CSVMaintainer::write_mpc_cmd(double time, RobotData& robot_cmd)
{
    if (!debug_mode)
        return;

    // foot_cmd
    mpc_cmd_csv.resetContent();
    mpc_cmd_csv << time;

    set_vector(robot_cmd.pos, mpc_cmd_csv);
    set_vector(robot_cmd.orientation, mpc_cmd_csv);
    set_vector(robot_cmd.lin_vel, mpc_cmd_csv);
    set_vector(robot_cmd.ang_vel, mpc_cmd_csv);

    mpc_cmd_csv.writeToFile(mpc_cmd_addr, true);
}

void CSVMaintainer::write_wbc_cmd(double time, LegData& leg_cmd, RobotData& robot_cmd)
{
    if (!debug_mode)
        return;

    // foot_cmd
    wbc_cmd_csv.resetContent();
    wbc_cmd_csv << time;

    set_vector(robot_cmd.pos, wbc_cmd_csv);
    set_vector(robot_cmd.orientation, wbc_cmd_csv);
    set_vector(robot_cmd.lin_vel, wbc_cmd_csv);
    set_vector(robot_cmd.ang_vel, wbc_cmd_csv);

    set_vector(leg_cmd.r1_pos, wbc_cmd_csv);
    set_vector(leg_cmd.l1_pos, wbc_cmd_csv);
    set_vector(leg_cmd.r2_pos, wbc_cmd_csv);
    set_vector(leg_cmd.l2_pos, wbc_cmd_csv);

    set_vector(leg_cmd.r1_vel, wbc_cmd_csv);
    set_vector(leg_cmd.l1_vel, wbc_cmd_csv);
    set_vector(leg_cmd.r2_vel, wbc_cmd_csv);
    set_vector(leg_cmd.l2_vel, wbc_cmd_csv);

    set_vector(leg_cmd.r1_grf, wbc_cmd_csv);
    set_vector(leg_cmd.l1_grf, wbc_cmd_csv);
    set_vector(leg_cmd.r2_grf, wbc_cmd_csv);
    set_vector(leg_cmd.l2_grf, wbc_cmd_csv);

    wbc_cmd_csv.writeToFile(wbc_cmd_addr, true);
}

void CSVMaintainer::write_imu_data(double time, ImuData &imu_data)
{
    imu_data_csv.resetContent();
    imu_data_csv << time;

    set_vector(imu_data.orientation_euler, imu_data_csv);
    set_vector(imu_data.orientation_quaternion, imu_data_csv);
    set_vector(imu_data.ang_vel, imu_data_csv);
    set_vector(imu_data.lin_accel, imu_data_csv);

    imu_data_csv.writeToFile(imu_data_addr, true);
}

void CSVMaintainer::write_odometry(double time, Odometry& odometry)
{
    odometry_csv.resetContent();
    odometry_csv << time;

    set_vector(odometry.position, odometry_csv);
    set_vector(odometry.orientation_euler, odometry_csv);
    set_vector(odometry.lin_vel, odometry_csv);
    set_vector(odometry.ang_vel, odometry_csv);

    odometry_csv.writeToFile(odometry_addr, true);
}

void CSVMaintainer::write_enable(double time, bool leg_controller_enable, bool leg_controller_reset,
                                bool locomotion_enable, bool locomotion_reset,
                                bool action_ctr_enable, bool action_ctr_reset)
{
    enable_csv.resetContent();
    enable_csv << time;
    enable_csv << leg_controller_enable << leg_controller_reset;
    enable_csv << locomotion_enable << locomotion_reset;
    enable_csv << action_ctr_enable << action_ctr_reset;

    // cout << leg_controller_enable << leg_controller_reset << endl;
    // cout << locomotion_enable << locomotion_reset << endl;
    // cout << action_ctr_enable << action_ctr_reset << endl;
    // cout << "=========" << endl;

    enable_csv.writeToFile(enable_addr, true);
}

void CSVMaintainer::write_robot_state(double time, RobotData& body_state, LegData& leg_state)
{
    // foot_cmd
    robot_state_csv.resetContent();
    robot_state_csv << time;

    set_vector(body_state.pos, robot_state_csv);
    set_vector(body_state.orientation, robot_state_csv);
    set_vector(body_state.lin_vel, robot_state_csv);
    set_vector(body_state.ang_vel, robot_state_csv);

    set_vector(leg_state.r1_pos, robot_state_csv);
    set_vector(leg_state.l1_pos, robot_state_csv);
    set_vector(leg_state.r2_pos, robot_state_csv);
    set_vector(leg_state.l2_pos, robot_state_csv);

    set_vector(leg_state.r1_vel, robot_state_csv);
    set_vector(leg_state.l1_vel, robot_state_csv);
    set_vector(leg_state.r2_vel, robot_state_csv);
    set_vector(leg_state.l2_vel, robot_state_csv);

    set_vector(leg_state.r1_grf, robot_state_csv);
    set_vector(leg_state.l1_grf, robot_state_csv);
    set_vector(leg_state.r2_grf, robot_state_csv);
    set_vector(leg_state.l2_grf, robot_state_csv);

    set_vector(leg_state.contacts, 4, robot_state_csv);

    robot_state_csv.writeToFile(robot_state_addr, true);
}

void CSVMaintainer::write_robot_state_check(double time, RobotData& body_state_check, LegData& leg_state_check)
{
    // foot_cmd
    robot_state_check_csv.resetContent();
    robot_state_check_csv << time;

    set_vector(body_state_check.pos, robot_state_check_csv);
    set_vector(body_state_check.orientation, robot_state_check_csv);
    set_vector(body_state_check.lin_vel, robot_state_check_csv);
    set_vector(body_state_check.ang_vel, robot_state_check_csv);

    set_vector(leg_state_check.r1_pos, robot_state_check_csv);
    set_vector(leg_state_check.l1_pos, robot_state_check_csv);
    set_vector(leg_state_check.r2_pos, robot_state_check_csv);
    set_vector(leg_state_check.l2_pos, robot_state_check_csv);

    set_vector(leg_state_check.r1_vel, robot_state_check_csv);
    set_vector(leg_state_check.l1_vel, robot_state_check_csv);
    set_vector(leg_state_check.r2_vel, robot_state_check_csv);
    set_vector(leg_state_check.l2_vel, robot_state_check_csv);

    set_vector(leg_state_check.r1_grf, robot_state_check_csv);
    set_vector(leg_state_check.l1_grf, robot_state_check_csv);
    set_vector(leg_state_check.r2_grf, robot_state_check_csv);
    set_vector(leg_state_check.l2_grf, robot_state_check_csv);

    set_vector(leg_state_check.contacts, 4, robot_state_check_csv);

    robot_state_check_csv.writeToFile(robot_state_check_addr, true);
}

void CSVMaintainer::write_robot_cmd(double time, RobotData& body_cmd)
{
    // foot_cmd
    robot_cmd_csv.resetContent();
    robot_cmd_csv << time;

    set_vector(body_cmd.pos, robot_cmd_csv);
    set_vector(body_cmd.orientation, robot_cmd_csv);
    set_vector(body_cmd.lin_vel, robot_cmd_csv);
    set_vector(body_cmd.ang_vel, robot_cmd_csv);

    robot_cmd_csv.writeToFile(robot_cmd_addr, true);
}

void CSVMaintainer::write_phase_sig(double t, Vector4i& phase, Vector4d& phi)
{
    if (!debug_mode)
        return;

    // foot_cmd
    phase_sig_csv.resetContent();
    phase_sig_csv << t;

    set_vector(phase, phase_sig_csv);
    set_vector(phi, phase_sig_csv);

    phase_sig_csv.writeToFile(phase_sig_addr, true);
}

void CSVMaintainer::set_vector(vector<bool> &data, int size, CSVWriter &csv)
{
    for (int i=0; i<size; i++)
        csv << data[i];
}

void CSVMaintainer::set_vector(VectorXd &data, int size, CSVWriter &csv)
{
    for (int i=0; i<size; i++)
    {
        if (std::abs(data(i)) < 1e-4)
            csv << 0.0;
        else
            csv << data(i);
    }
}

void CSVMaintainer::set_vector(Vector3d &data, CSVWriter &csv)
{
    for (int i=0; i<3; i++)
    {
        if (std::abs(data(i)) < 1e-4)
            csv << 0.0;
        else
            csv << data(i);
    }
}

void CSVMaintainer::set_vector(Vector4d &data, CSVWriter &csv)
{
    for (int i=0; i<4; i++)
    {
        if (std::abs(data(i)) < 1e-4)
            csv << 0.0;
        else
            csv << data(i);
    }
        
}

void CSVMaintainer::set_vector(Vector4i &data, CSVWriter &csv)
{
    for (int i=0; i<4; i++)
    {
        if (std::abs(data(i)) < 1e-4)
            csv << 0.0;
        else
            csv << data(i);
    }
}
