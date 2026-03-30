#include "robot_mode_controller/lcm_data_exchange.hpp"
#include <unistd.h>

#define KT 0.74
#define GEAR_RATIO 10.0

string GetEnv( const std::string & var ) 
{
    const char * val = std::getenv( var.c_str() );
    if ( val == nullptr ) { // invalid to assign nullptr to std::string
        return "";
    }
    else {
        return val;
    }
}

LCMExchanger::LCMExchanger()
{

    if(!servo_state_subscriber.good())
        return;
    if(!robot_state_subscriber.good())
        return;
    if(!enable_publisher.good())
        return;
    if(!servo_cmd_publisher.good())
        return;

    string config_address = GetEnv("CONFIGPATH");
    config_address += "/channels.yaml";

    YAML::Node channel_config = YAML::LoadFile(config_address);
    // YAML::Node channel_config = YAML::LoadFile("/home/user/mors_mpc_control/config/channels.yaml");
    enable_channel = channel_config["enable"].as<string>();
    robot_cmd_channel = channel_config["robot_cmd"].as<string>();
    wbc_cmd_channel = channel_config["wbc_cmd"].as<string>();
    servo_cmd_channel = channel_config["servo_cmd"].as<string>();
    servo_state_channel = channel_config["servo_state"].as<string>();
    robot_state_channel = channel_config["robot_state"].as<string>();
    phase_signal_channel = channel_config["gait_phase"].as<string>();
    gait_params_channel = channel_config["gait_params"].as<string>();

    // gear_ratio = GEAR_RATIO;

    leg_controller_en = false;
    leg_controller_reset = true;

    servo_pos.setZero(12);
    servo_vel.setZero(12);
    servo_torq.setZero(12);

    robot_state.ang_vel.setZero(3);
    robot_state.lin_vel.setZero(3);
    robot_state.orientation.setZero(3);
    robot_state.pos.setZero(3);
    robot_cmd.ang_vel.setZero(3);
    robot_cmd.lin_vel.setZero(3);
    robot_cmd.orientation.setZero(3);
    robot_cmd.pos.setZero(3);
    leg_state.contacts = {false, false, false, false};
    leg_state.r1_pos.setZero(3);
    leg_state.l1_pos.setZero(3);
    leg_state.r2_pos.setZero(3);
    leg_state.l2_pos.setZero(3);
    leg_state.r1_vel.setZero(3);
    leg_state.l1_vel.setZero(3);
    leg_state.r2_vel.setZero(3);
    leg_state.l2_vel.setZero(3);
}

void LCMExchanger::start_exchanger()
{
    thRobotState = make_unique<thread> (&LCMExchanger::robotStateThread, this);
    thServoState = make_unique<thread> (&LCMExchanger::servoStateThread, this);
}

void LCMExchanger::robotStateHandler(const lcm::ReceiveBuffer* rbuf,
                                    const std::string& chan,
                                    const mors_msgs::robot_state_msg* msg)
{
    (void)rbuf;
    (void)chan;

    for (int i=0; i<3; i++)
    {
        leg_state.r1_pos(i) = msg->legs.r1_pos[i];
        leg_state.l1_pos(i) = msg->legs.l1_pos[i];
        leg_state.r2_pos(i) = msg->legs.r2_pos[i];
        leg_state.l2_pos(i) = msg->legs.l2_pos[i];

        // leg_state.r1_vel(i) = msg->legs.r1_vel[i];
        // leg_state.l1_vel(i) = msg->legs.l1_vel[i];
        // leg_state.r2_vel(i) = msg->legs.r2_vel[i];
        // leg_state.l2_vel(i) = msg->legs.l2_vel[i];

        // leg_state.r1_grf(i) = msg->legs.r1_grf[i];
        // leg_state.l1_grf(i) = msg->legs.l1_grf[i];
        // leg_state.r2_grf(i) = msg->legs.r2_grf[i];
        // leg_state.l2_grf(i) = msg->legs.l2_grf[i];

        leg_state.contacts[i] = msg->legs.contact_states[i];

        robot_state.pos(i) = msg->body.position[i];
        robot_state.orientation(i) = msg->body.orientation[i];
        robot_state.lin_vel(i) = msg->body.lin_vel[i];
        robot_state.ang_vel(i) = msg->body.ang_vel[i];
    }
    leg_state.contacts[3] = msg->legs.contact_states[3];
}

void LCMExchanger::servoStateHandler(const lcm::ReceiveBuffer* rbuf,
                            const std::string& chan,
                            const mors_msgs::servo_state_msg* msg)
{
    (void)rbuf;
    (void)chan;

    // cout << "I got servo state data!" << endl;
    for (int i=0; i<12; i++)
    {
        // if (i == 2 || i == 5 || i == 8 || i == 11)
        // {
        //     gear_ratio = GEAR_RATIO;
        //     // cout << "hey" << endl;
        // }
        // else
        //     gear_ratio = GEAR_RATIO;
        servo_pos(i) = msg->position[i];
        servo_vel(i) = msg->velocity[i];
        servo_torq(i) = msg->torque[i];// * KT / gear_ratio;
    }
}

void LCMExchanger::robotStateThread()
{   
    robot_state_subscriber.subscribe(robot_state_channel, &LCMExchanger::robotStateHandler, this);
    while(true)
    {
        robot_state_subscriber.handle();
        // cout << "grf thread" << endl;
        // sleep(0.001);
    }
}

void LCMExchanger::servoStateThread()
{
    servo_state_subscriber.subscribe(servo_state_channel, &LCMExchanger::servoStateHandler, this);
    while(true)
    {
        servo_state_subscriber.handle();
        // cout << "servo_state thread" << endl;
        // sleep(0.001);
    }
}

void LCMExchanger::sendServoCmd(VectorXd &position, VectorXd &velocity, VectorXd &torque, VectorXd &kp, VectorXd &kd)
{
    for (int i=0; i<12; i++)
    {
        // if (i == 2 || i == 5 || i == 8 || i == 11)
        // {
        //     gear_ratio = GEAR_RATIO;
        //     // cout << "hey" << endl;
        // }
        // else
        //     gear_ratio = GEAR_RATIO;
        // cout << i << ": " << gear_ratio << endl;
        servoCmdMsg.position[i] = position(i);
        servoCmdMsg.velocity[i] = velocity(i);
        servoCmdMsg.torque[i] = torque(i);// * gear_ratio / KT;
        servoCmdMsg.kp[i] = kp(i);
        servoCmdMsg.kd[i] = kd(i);
    }
    // for (int i=0; i<12; i++)
    // {
    //     servoCmdMsg.position[i] = 0.0;
    //     servoCmdMsg.velocity[i] = 0.0;
    //     servoCmdMsg.torque[i] = 0.0;
    //     servoCmdMsg.kp[i] = 0.0;
    //     servoCmdMsg.kd[i] = 0.0;
    // }
    // cout << torque << endl;
    // servoCmdMsg.torque[2] = 1.0;
    servo_cmd_publisher.publish(servo_cmd_channel, &servoCmdMsg);
}

void LCMExchanger::sendEnableData(bool &locomotion_en, bool &locomotion_reset,
                                    bool &leg_controller_en, bool &leg_controller_reset)
{
    enableMsg.locomotion_en = locomotion_en;
    enableMsg.locomotion_reset = locomotion_reset;

    enableMsg.leg_controller_en = leg_controller_en;
    enableMsg.leg_controller_reset = leg_controller_reset;

    enable_publisher.publish(enable_channel, &enableMsg);
}

void LCMExchanger::sendWbcCmd(LegData &leg_data, RobotData &robot_data)
{
    for (int i=0; i<3; i++)
    {
        wbcCmdMsg.body.position[i] = robot_data.pos(i);
        wbcCmdMsg.body.orientation_euler[i] = robot_data.orientation(i);
        wbcCmdMsg.body.lin_vel[i] = robot_data.lin_vel(i);
        wbcCmdMsg.body.ang_vel[i] = robot_data.ang_vel(i);

        wbcCmdMsg.legs.r1_pos[i] = leg_data.r1_pos(i);
        wbcCmdMsg.legs.l1_pos[i] = leg_data.l1_pos(i);
        wbcCmdMsg.legs.r2_pos[i] = leg_data.r2_pos(i);
        wbcCmdMsg.legs.l2_pos[i] = leg_data.l2_pos(i);

        wbcCmdMsg.legs.r1_vel[i] = leg_data.r1_vel(i);
        wbcCmdMsg.legs.l1_vel[i] = leg_data.l1_vel(i);
        wbcCmdMsg.legs.r2_vel[i] = leg_data.r2_vel(i);
        wbcCmdMsg.legs.l2_vel[i] = leg_data.l2_vel(i);

        wbcCmdMsg.legs.r1_grf[i] = leg_data.r1_grf(i);
        wbcCmdMsg.legs.l1_grf[i] = leg_data.l1_grf(i);
        wbcCmdMsg.legs.r2_grf[i] = leg_data.r2_grf(i);
        wbcCmdMsg.legs.l2_grf[i] = leg_data.l2_grf(i);
    }
    wbc_cmd_publisher.publish(wbc_cmd_channel, &wbcCmdMsg);
}

void LCMExchanger::sendPhaseSig(vector<int>& phase, vector<double>& phi, double t)
{
    for (int i = 0; i < 4; i++)
    {
        phaseSigMsg.phase[i] = phase[i];
        phaseSigMsg.phi[i] = phi[i];
    }
    // phaseSigMsg.num = static_cast<int16_t>(gait_table.size());
    // phaseSigMsg.gait_table.resize(static_cast<int16_t>(gait_table.size()));
    // for (int16_t i = 0; i < static_cast<int16_t>(gait_table.size()); i++)
    // {
    //     phaseSigMsg.gait_table[i] = static_cast<int16_t>(gait_table[i]);
    // }
    phaseSigMsg.t = t;
    phase_sig_publisher.publish(phase_signal_channel, &phaseSigMsg);
}

void LCMExchanger::sendGaitParams(double& t_st, double& t_sw, vector<double>& gait_type, bool& standing, double& stride_height)
{
    gaitPrmsMsg.t_st = t_st;
    gaitPrmsMsg.t_sw = t_sw;
    gaitPrmsMsg.standing = standing;
    gaitPrmsMsg.stride_height = stride_height;

    for (int i = 0; i < 4; i++)
        gaitPrmsMsg.gait_type[i] = gait_type[i];

    gait_params_publisher.publish(gait_params_channel, &gaitPrmsMsg);
}

void LCMExchanger::sendRobotCmd(RobotData& robot_cmd, vector<bool> &active_legs, int8_t& adaptation_type)
{
    robotCmdMsg.adaptation_type = adaptation_type;
    for (int i = 0; i < 4; i++)
    {
        robotCmdMsg.active_legs[i] = active_legs[i];
    }
    for (int i = 0; i < 3; i++)
    {
        robotCmdMsg.cmd_vel[i] = robot_cmd.lin_vel[i];
        robotCmdMsg.cmd_vel[i+3] = robot_cmd.ang_vel[i];
        robotCmdMsg.cmd_pose[i] = robot_cmd.pos[i];
        robotCmdMsg.cmd_pose[i+3] = robot_cmd.orientation[i];
    }
    robot_cmd_publisher.publish(robot_cmd_channel, &robotCmdMsg);
}

RobotData LCMExchanger::getBodyState()
{
    return robot_state;
}

LegData LCMExchanger::getLegState()
{
    return leg_state;
}

void LCMExchanger::getServoStateData(VectorXd &position, VectorXd &velocity, VectorXd &torque)
{
    position = servo_pos;
    velocity = servo_vel;
    torque = servo_torq;
}

LCMExchanger::~LCMExchanger()
{
    bool en = false;
    bool reset = true;
    sendEnableData(en, reset, en, reset);
}
