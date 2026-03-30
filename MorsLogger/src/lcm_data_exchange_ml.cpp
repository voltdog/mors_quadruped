#include "lcm_data_exchange_ml.hpp"
#include <unistd.h>

// #define KT 0.74
// #define GEAR_RATIO 10.0

LCMExchanger::LCMExchanger(bool debug_mode)
    : debug_mode(debug_mode)
{
    if(!mpc_cmd_subscriber.good())
        return;
    if(!wbc_cmd_subscriber.good())
        return;
    if(!servo_state_subscriber.good())
        return;
    if(!imu_subscriber.good())
        return;
    if(!enable_subscriber.good())
        return;
    if(!servo_cmd_subscriber.good())
        return;
    if(!odometry_subscriber.good())
        return;
    // if(!servo_state_filt_subscriber.good())
    //     return;
    if(!robot_state_subscriber.good())
        return;
    if(!robot_state_check_subscriber.good())
        return;
    if(!robot_cmd_subscriber.good())
        return;
    if(!gait_phase_sig_subscriber.good())
        return;
    if(!contact_sensor_subscriber.good())
        return;

    // char cwd[PATH_MAX];
    // if (getcwd(cwd, sizeof(cwd)) != NULL) {
    //     // print the current working directory
    //     // cout << "Current working directory: " << cwd << endl;
    // } 
    string config_address = mors_sys::GetEnv("CONFIGPATH");//cwd;
    config_address += "/channels.yaml";
    // cout << config_address << endl;

    YAML::Node channel_config = YAML::LoadFile(config_address);//"/home/user/mors_mpc_control/config/channels.yaml");
    enable_channel = channel_config["enable"].as<string>();

    imu_channel = channel_config["imu_data"].as<string>();
    odometry_channel = channel_config["odometry"].as<string>();
    contact_state_channel = channel_config["contact_state"].as<string>();

    robot_cmd_channel = channel_config["robot_cmd"].as<string>();
    mpc_cmd_channel = channel_config["mpc_cmd"].as<string>();
    wbc_cmd_channel = channel_config["wbc_cmd"].as<string>();
    servo_cmd_channel = channel_config["servo_cmd"].as<string>();

    servo_state_channel = channel_config["servo_state"].as<string>();
    robot_state_channel = channel_config["robot_state"].as<string>();
    robot_state_check_channel = "ROBOT_STATE_CHECK";
    
    gait_phase_sig_channel = channel_config["gait_phase"].as<string>();

    leg_controller_enable = false;
    leg_controller_reset = true;
    locomotion_enable = false;
    locomotion_reset = true;
    action_ctr_enable = false;
    action_ctr_reset = true;
   
    servo_state.pos.resize(12);
    servo_state.vel.resize(12);
    servo_state.torq.resize(12);
    servo_state.pos.setZero();
    servo_state.vel.setZero();
    servo_state.torq.setZero();

    servo_state_filt.pos.resize(12);
    servo_state_filt.vel.resize(12);
    servo_state_filt.torq.resize(12);
    servo_state_filt.pos.setZero();
    servo_state_filt.vel.setZero();
    servo_state_filt.torq.setZero();
    
    servo_cmd.pos.resize(12);
    servo_cmd.vel.resize(12);
    servo_cmd.torq.resize(12);
    servo_cmd.kp.resize(12);
    servo_cmd.kd.resize(12);
    servo_cmd.pos.setZero();
    servo_cmd.vel.setZero();
    servo_cmd.torq.setZero();
    servo_cmd.kp.setZero();
    servo_cmd.kd.setZero();

    leg_state.contacts.resize(4);
    leg_state.contacts = {false, false, false, false};

    leg_state_check.contacts.resize(4);
    leg_state_check.contacts = {false, false, false, false};

    contact_state.resize(4);

    phase.setZero();
    phi.setZero();
}

void LCMExchanger::start_exchanger()
{
    if (debug_mode)
    {
        thMpcCmd = make_unique<thread> (&LCMExchanger::mpcCmdThread, this);
        thWbcCmd = make_unique<thread> (&LCMExchanger::wbcCmdThread, this);
        thGaitPhaseSig = make_unique<thread> (&LCMExchanger::phaseSigThread, this);
    }
    thImu = make_unique<thread> (&LCMExchanger::imuThread, this);
    thServoState = make_unique<thread> (&LCMExchanger::servoStateThread, this);
    thServoCmd = make_unique<thread> (&LCMExchanger::servoCmdThread, this);
    // thEnable = make_unique<thread> (&LCMExchanger::enableThread, this);
    // thCtrlType = make_unique<thread> (&LCMExchanger::ctrlTypeThread, this);
    thOdometry = make_unique<thread> (&LCMExchanger::odometryThread, this);
    // thServoStateFilt = make_unique<thread> (&LCMExchanger::servoStateFiltThread, this);
    thRobotState = make_unique<thread> (&LCMExchanger::robotStateThread, this);
    thRobotStateCheck = make_unique<thread> (&LCMExchanger::robotStateCheckThread, this);
    thRobotCmd = make_unique<thread> (&LCMExchanger::robotCmdThread, this);
    thContactSensor = make_unique<thread> (&LCMExchanger::contactSensorThread, this);
}

void LCMExchanger::contactSensorHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::contact_sensor_msg* msg)
{
    // cout << "I got foot_cmd!" << endl;
    for (int i=0; i<4; i++)
        contact_state[i] = msg->contact_states[i];
}

void LCMExchanger::mpcCmdHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::mpc_cmd_msg* msg)
{
    // cout << "I got foot_cmd!" << endl;
    for (int i=0; i<3; i++)
    {
        mpc_body_cmd.pos(i) = msg->cmd_pose[i];
        mpc_body_cmd.orientation(i) = msg->cmd_pose[i+3];
        mpc_body_cmd.lin_vel(i) = msg->cmd_vel[i];
        mpc_body_cmd.ang_vel(i) = msg->cmd_vel[i+3];
    }
} 

void LCMExchanger::wbcCmdHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::wbc_cmd_msg* msg)
{
    // cout << "I got foot_cmd!" << endl;
    for (int i=0; i<3; i++)
    {
        wbc_body_cmd.pos(i) = msg->body.position[i];
        wbc_body_cmd.orientation(i) = msg->body.orientation_euler[i];
        wbc_body_cmd.lin_vel(i) = msg->body.lin_vel[i];
        wbc_body_cmd.ang_vel(i) = msg->body.ang_vel[i];

        wbc_leg_cmd.r1_pos(i) = msg->legs.r1_pos[i];
        wbc_leg_cmd.l1_pos(i) = msg->legs.l1_pos[i];
        wbc_leg_cmd.r2_pos(i) = msg->legs.r2_pos[i];
        wbc_leg_cmd.l2_pos(i) = msg->legs.l2_pos[i];

        wbc_leg_cmd.r1_vel(i) = msg->legs.r1_vel[i];
        wbc_leg_cmd.l1_vel(i) = msg->legs.l1_vel[i];
        wbc_leg_cmd.r2_vel(i) = msg->legs.r2_vel[i];
        wbc_leg_cmd.l2_vel(i) = msg->legs.l2_vel[i];

        wbc_leg_cmd.r1_grf(i) = msg->legs.r1_grf[i];
        wbc_leg_cmd.r2_grf(i) = msg->legs.r2_grf[i];
        wbc_leg_cmd.l1_grf(i) = msg->legs.l1_grf[i];
        wbc_leg_cmd.l2_grf(i) = msg->legs.l2_grf[i];
    }
}

void LCMExchanger::imuHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan, 
                            const mors_msgs::imu_lcm_data* msg)
{
    for (int i=0; i<3; i++)
    {
        imu_data.orientation_euler(i) = msg->orientation_euler[i];
        imu_data.orientation_quaternion(i) = msg->orientation_quaternion[i];
        imu_data.ang_vel(i) = msg->angular_velocity[i];
        imu_data.lin_accel(i) = msg->linear_acceleration[i];
    }
    imu_data.orientation_quaternion(3) = msg->orientation_quaternion[3];
}

void LCMExchanger::servoStateHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::servo_state_msg* msg)
{
    for (int i=0; i<12; i++)
    {
        servo_state.pos(i) = msg->position[i];
        servo_state.vel(i) = msg->velocity[i];
        servo_state.torq(i) = -msg->torque[i];// * KT / gear_ratio;
    }
}

void LCMExchanger::servoStateFiltHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::servo_state_msg* msg)
{
    for (int i=0; i<12; i++)
    {
        servo_state_filt.pos(i) = msg->position[i];
        servo_state_filt.vel(i) = msg->velocity[i];
        servo_state_filt.torq(i) = -msg->torque[i];// * KT / gear_ratio;
    }
}

void LCMExchanger::servoCmdHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::servo_cmd_msg* msg)
{
    for (int i=0; i<12; i++)
    {
        servo_cmd.pos(i) = msg->position[i];
        servo_cmd.vel(i) = msg->velocity[i];
        servo_cmd.torq(i) = msg->torque[i];// * KT / gear_ratio;
        servo_cmd.kp(i) = msg->kp[i];
        servo_cmd.kd(i) = msg->kd[i];
    }
}

void LCMExchanger::enableHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::enable_msg* msg)
{
    leg_controller_enable = msg->leg_controller_en;
    leg_controller_reset = msg->leg_controller_reset;

    locomotion_enable = msg->locomotion_en;
    locomotion_reset = msg->locomotion_reset;
}

void LCMExchanger::odometryHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan, 
    const mors_msgs::odometry_msg* msg)
{
    for (int i=0; i<3; i++)
    {
        odometry.position(i) = msg->position[i];
        odometry.orientation_euler(i) = msg->orientation[i];
        odometry.lin_vel(i) = msg->lin_vel[i];
        odometry.ang_vel(i) = msg->ang_vel[i];
    }
}

void LCMExchanger::robotStateHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::robot_state_msg* msg)
{
    for (int i=0; i<3; i++)
    {
        leg_state.r1_pos(i) = msg->legs.r1_pos[i];
        leg_state.l1_pos(i) = msg->legs.l1_pos[i];
        leg_state.r2_pos(i) = msg->legs.r2_pos[i];
        leg_state.l2_pos(i) = msg->legs.l2_pos[i];

        leg_state.r1_vel(i) = msg->legs.r1_vel[i];
        leg_state.l1_vel(i) = msg->legs.l1_vel[i];
        leg_state.r2_vel(i) = msg->legs.r2_vel[i];
        leg_state.l2_vel(i) = msg->legs.l2_vel[i];

        leg_state.r1_grf(i) = msg->legs.r1_grf[i];
        leg_state.l1_grf(i) = msg->legs.l1_grf[i];
        leg_state.r2_grf(i) = msg->legs.r2_grf[i];
        leg_state.l2_grf(i) = msg->legs.l2_grf[i];

        leg_state.contacts[i] = msg->legs.contact_states[i];

        body_state.pos(i) = msg->body.position[i];
        body_state.orientation(i) = msg->body.orientation[i];
        body_state.lin_vel(i) = msg->body.lin_vel[i];
        body_state.ang_vel(i) = msg->body.ang_vel[i];
    }
    leg_state.contacts[3] = msg->legs.contact_states[3];
}

void LCMExchanger::robotStateCheckHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::robot_state_msg* msg)
{
    for (int i=0; i<3; i++)
    {
        leg_state_check.r1_pos(i) = msg->legs.r1_pos[i];
        leg_state_check.l1_pos(i) = msg->legs.l1_pos[i];
        leg_state_check.r2_pos(i) = msg->legs.r2_pos[i];
        leg_state_check.l2_pos(i) = msg->legs.l2_pos[i];

        leg_state_check.r1_vel(i) = msg->legs.r1_vel[i];
        leg_state_check.l1_vel(i) = msg->legs.l1_vel[i];
        leg_state_check.r2_vel(i) = msg->legs.r2_vel[i];
        leg_state_check.l2_vel(i) = msg->legs.l2_vel[i];

        leg_state_check.r1_grf(i) = msg->legs.r1_grf[i];
        leg_state_check.l1_grf(i) = msg->legs.l1_grf[i];
        leg_state_check.r2_grf(i) = msg->legs.r2_grf[i];
        leg_state_check.l2_grf(i) = msg->legs.l2_grf[i];

        leg_state_check.contacts[i] = msg->legs.contact_states[i];

        body_state_check.pos(i) = msg->body.position[i];
        body_state_check.orientation(i) = msg->body.orientation[i];
        body_state_check.lin_vel(i) = msg->body.lin_vel[i];
        body_state_check.ang_vel(i) = msg->body.ang_vel[i];
    }
    leg_state_check.contacts[3] = msg->legs.contact_states[3];
}

void LCMExchanger::robotCmdHandler(const lcm::ReceiveBuffer* rbuf,
    const std::string& chan,
    const mors_msgs::robot_cmd_msg* msg)
{
    for (int i=0; i<3; i++)
    {
        body_cmd.pos(i) = msg->cmd_pose[i];
        body_cmd.orientation(i) = msg->cmd_pose[i+3];
        body_cmd.lin_vel(i) = msg->cmd_vel[i];
        body_cmd.ang_vel(i) = msg->cmd_vel[i+3];
    }
}

void LCMExchanger::phaseSigHandler(const lcm::ReceiveBuffer* rbuf, 
    const std::string& chan, 
    const mors_msgs::phase_signal_msg* msg)
{
    for (int i=0; i<4; i++)
    {
        phase(i) = msg->phase[i];
        phi(i) = msg->phi[i];
    }
}

void LCMExchanger::contactSensorThread()
{   
    contact_sensor_subscriber.subscribe(contact_state_channel, &LCMExchanger::contactSensorHandler, this);
    while(true)
    {
        contact_sensor_subscriber.handle();
    }
}

void LCMExchanger::mpcCmdThread()
{   
    mpc_cmd_subscriber.subscribe(mpc_cmd_channel, &LCMExchanger::mpcCmdHandler, this);
    while(true)
    {
        mpc_cmd_subscriber.handle();
    }
}

void LCMExchanger::wbcCmdThread()
{   
    wbc_cmd_subscriber.subscribe(wbc_cmd_channel, &LCMExchanger::wbcCmdHandler, this);
    while(true)
    {
        wbc_cmd_subscriber.handle();
    }
}

void LCMExchanger::imuThread()
{
    imu_subscriber.subscribe(imu_channel, &LCMExchanger::imuHandler, this);
    while(true)
    {
        imu_subscriber.handle();
    }
}

void LCMExchanger::servoStateThread()
{
    servo_state_subscriber.subscribe(servo_state_channel, &LCMExchanger::servoStateHandler, this);
    while(true)
    {
        servo_state_subscriber.handle();
    }
}

void LCMExchanger::servoCmdThread()
{
    servo_cmd_subscriber.subscribe(servo_cmd_channel, &LCMExchanger::servoCmdHandler, this);
    while(true)
    {
        servo_cmd_subscriber.handle();
    }
}

void LCMExchanger::enableThread()
{
    enable_subscriber.subscribe(enable_channel, &LCMExchanger::enableHandler, this);
    while(true)
    {
        enable_subscriber.handle();
    }
}

void LCMExchanger::odometryThread()
{   
    odometry_subscriber.subscribe(odometry_channel, &LCMExchanger::odometryHandler, this);
    while(true)
    {
        odometry_subscriber.handle();
    }
}

void LCMExchanger::robotStateThread()
{
    robot_state_subscriber.subscribe(robot_state_channel, &LCMExchanger::robotStateHandler, this);
    while(true)
    {
        robot_state_subscriber.handle();
    }
}

void LCMExchanger::robotStateCheckThread()
{
    robot_state_check_subscriber.subscribe(robot_state_check_channel, &LCMExchanger::robotStateCheckHandler, this);
    while(true)
    {
        robot_state_check_subscriber.handle();
    }
}

void LCMExchanger::robotCmdThread()
{
    robot_cmd_subscriber.subscribe(robot_cmd_channel, &LCMExchanger::robotCmdHandler, this);
    while(true)
    {
        robot_cmd_subscriber.handle();
    }
}

void LCMExchanger::phaseSigThread()
{
    gait_phase_sig_subscriber.subscribe(gait_phase_sig_channel, &LCMExchanger::phaseSigHandler, this);
    while(true)
    {
        gait_phase_sig_subscriber.handle();
    }
}

void LCMExchanger::getMpcCmdData(RobotData &mpc_body_cmd)
{
    mpc_body_cmd = this->mpc_body_cmd;
}

void LCMExchanger::getWbcCmdData(LegData &wbc_leg_cmd, RobotData &wbc_body_cmd)
{
    wbc_leg_cmd = this->wbc_leg_cmd;
    wbc_body_cmd = this->wbc_body_cmd;
}

ImuData LCMExchanger::getImuData()
{
    return imu_data;
}

vector<bool> LCMExchanger::getContactSensorData()
{
    return contact_state;
}

ServoData LCMExchanger::getServoStateData()
{
    return servo_state;
}

ServoData LCMExchanger::getServoStateFiltData()
{
    return servo_state_filt;
}

ServoData LCMExchanger::getServoCmdData()
{
    return servo_cmd;
}

Odometry LCMExchanger::getOdometry()
{
    return odometry;
}

RobotData LCMExchanger::getBodyState()
{
    return body_state;
}

LegData LCMExchanger::getLegState()
{
    return leg_state;
}

LegData LCMExchanger::getLegStateCheck()
{
    return leg_state_check;
}
RobotData LCMExchanger::getBodyStateCheck()
{
    return body_state_check;
}
RobotData LCMExchanger::getRobotCmd()
{
    return body_cmd;
}
void LCMExchanger::getPhaseSig(Vector4i& phase, Vector4d& phi)
{
    phase = this->phase;
    phi = this->phi;
}

void LCMExchanger::getEnableData(bool &leg_controller_en, bool &leg_controller_reset,
                                bool &locomotion_en, bool &locomotion_reset,
                                bool &action_ctr_en, bool &action_ctr_reset)
{
    leg_controller_en = this->leg_controller_enable;
    leg_controller_reset = this->leg_controller_reset;

    locomotion_en = this->locomotion_enable;
    locomotion_reset = this->locomotion_reset;

    action_ctr_en = this->action_ctr_enable;
    action_ctr_reset = this->action_ctr_reset;
}

LCMExchanger::~LCMExchanger()
{

}
