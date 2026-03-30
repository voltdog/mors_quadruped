#include "LcmDataExchange.hpp"

#include <cmath>
#include <Eigen/Geometry>
#include <unistd.h>

namespace {

RobotData makeZeroRobotData()
{
    RobotData data;
    data.pos.setZero();
    data.lin_vel.setZero();
    data.orientation.setZero();
    data.orientation_quaternion << 0.0, 0.0, 0.0, 1.0;
    data.ang_vel.setZero();
    return data;
}

LegData makeZeroLegData()
{
    LegData data;
    data.r1_grf.setZero();
    data.l1_grf.setZero();
    data.r2_grf.setZero();
    data.l2_grf.setZero();
    data.r1_pos.setZero();
    data.l1_pos.setZero();
    data.r2_pos.setZero();
    data.l2_pos.setZero();
    data.r1_vel.setZero();
    data.l1_vel.setZero();
    data.r2_vel.setZero();
    data.l2_vel.setZero();
    data.r1_acc.setZero();
    data.l1_acc.setZero();
    data.r2_acc.setZero();
    data.l2_acc.setZero();
    data.r1_kp.setZero();
    data.l1_kp.setZero();
    data.r2_kp.setZero();
    data.l2_kp.setZero();
    data.r1_kd.setZero();
    data.l1_kd.setZero();
    data.r2_kd.setZero();
    data.l2_kd.setZero();
    data.contacts = std::vector<bool>(NUM_LEGS, false);
    return data;
}

ServoStateData makeZeroServoStateData()
{
    ServoStateData data;
    return data;
}

template<typename Derived>
void replace_non_finite(Eigen::MatrixBase<Derived>& value)
{
    for (Eigen::Index i = 0; i < value.size(); ++i) {
        if (!std::isfinite(value.derived().coeff(i))) {
            value.derived().coeffRef(i) = 0.0;
        }
    }
}

Eigen::Vector4d sanitize_quaternion_xyzw(const Eigen::Vector4d& raw_quaternion)
{
    Eigen::Vector4d quaternion = raw_quaternion;
    replace_non_finite(quaternion);

    const double quaternion_norm = quaternion.norm();
    if (quaternion_norm < 1e-6) {
        quaternion << 0.0, 0.0, 0.0, 1.0;
        return quaternion;
    }

    quaternion /= quaternion_norm;
    return quaternion;
}

} // namespace

LCMExchanger::LCMExchanger()
    : robot_state(makeZeroRobotData()),
      robot_cmd(makeZeroRobotData()),
      leg_state(makeZeroLegData()),
      servo_state(makeZeroServoStateData())
{
    initialized =
        robot_cmd_subscriber.good() &&
        robot_state_subscriber.good() &&
        servo_state_subscriber.good() &&
        gait_params_subscriber.good() &&
        enable_subscriber.good() &&
        wbc_cmd_publisher.good() &&
        wbc_state_publisher.good() &&
        servo_cmd_publisher.good() &&
        phase_sig_publisher.good() &&
        robot_ref_publisher.good();

    if (!initialized) {
        return;
    }

    string config_address = mors_sys::GetEnv("CONFIGPATH");
    config_address += "/channels.yaml";

    YAML::Node channel_config = YAML::LoadFile(config_address);
    enable_channel = channel_config["enable"].as<string>();
    robot_cmd_channel = channel_config["robot_cmd"].as<string>();
    mpc_cmd_channel = channel_config["mpc_cmd"].as<string>();
    wbc_cmd_channel = channel_config["wbc_cmd"].as<string>();
    wbc_state_channel = channel_config["wbc_state"].as<string>();
    servo_cmd_channel = channel_config["servo_cmd"].as<string>();
    robot_state_channel = channel_config["robot_state"].as<string>();
    servo_state_channel = channel_config["servo_state"].as<string>();
    gait_params_channel = channel_config["gait_params"].as<string>();
    phase_signal_channel = channel_config["gait_phase"].as<string>();

    t_sw = 0.2;
    t_st = 0.3;
    gait_type = {0.0, 0.0, 0.0, 0.0};
    standing = true;
    stride_height = 0.06;
    active_legs = {true, true, true, true};
    adaptation_type = 0;

    locomotion_enable = false;
    leg_controller_enable = false;
    leg_controller_reset = true;

    for (int i = 0; i < 3; ++i) {
        wbcStateMsg.r1_grf[i] = 0.0f;
        wbcStateMsg.l1_grf[i] = 0.0f;
        wbcStateMsg.r2_grf[i] = 0.0f;
        wbcStateMsg.l2_grf[i] = 0.0f;
    }

    for (int i = 0; i < 12; ++i) {
        servoCmdMsg.position[i] = 0.0f;
        servoCmdMsg.velocity[i] = 0.0f;
        servoCmdMsg.torque[i] = 0.0f;
        servoCmdMsg.kp[i] = 0.0f;
        servoCmdMsg.kd[i] = 0.0f;
    }
}

void LCMExchanger::start_exchanger()
{
    thRobotCmd = make_unique<thread>(&LCMExchanger::robotCmdThread, this);
    thRobotState = make_unique<thread>(&LCMExchanger::robotStateThread, this);
    thServoState = make_unique<thread>(&LCMExchanger::servoStateThread, this);
    thEnable = make_unique<thread>(&LCMExchanger::enableThread, this);
    thGaitParams = make_unique<thread>(&LCMExchanger::gaitParamsThread, this);
}

void LCMExchanger::robotCmdHandler(const lcm::ReceiveBuffer*,
                                   const std::string&,
                                   const mors_msgs::robot_cmd_msg* msg)
{
    lock_guard<mutex> lock(robot_cmd_mutex);
    for (int i = 0; i < 3; ++i) {
        robot_cmd.pos(i) = msg->cmd_pose[i];
        robot_cmd.orientation(i) = msg->cmd_pose[i + 3];
        robot_cmd.lin_vel(i) = msg->cmd_vel[i];
        robot_cmd.ang_vel(i) = msg->cmd_vel[i + 3];
    }
    active_legs.assign(msg->active_legs, msg->active_legs + NUM_LEGS);
    adaptation_type = msg->adaptation_type;
}

void LCMExchanger::robotStateHandler(const lcm::ReceiveBuffer*,
                                     const std::string&,
                                     const mors_msgs::robot_state_msg* msg)
{
    lock_guard<mutex> lock(robot_state_mutex);
    for (int i = 0; i < 3; ++i) {
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

        robot_state.pos(i) = msg->body.position[i];
        robot_state.orientation(i) = msg->body.orientation[i];
        robot_state.orientation_quaternion(i) = msg->body.orientation_quaternion[i];
        robot_state.lin_vel(i) = msg->body.lin_vel[i];
        robot_state.ang_vel(i) = msg->body.ang_vel[i];
    }

    leg_state.contacts[3] = msg->legs.contact_states[3];
    robot_state.orientation_quaternion(3) = msg->body.orientation_quaternion[3];
    replace_non_finite(robot_state.pos);
    replace_non_finite(robot_state.orientation);
    replace_non_finite(robot_state.lin_vel);
    replace_non_finite(robot_state.ang_vel);
    robot_state.orientation_quaternion =
        sanitize_quaternion_xyzw(robot_state.orientation_quaternion);

    replace_non_finite(leg_state.r1_pos);
    replace_non_finite(leg_state.l1_pos);
    replace_non_finite(leg_state.r2_pos);
    replace_non_finite(leg_state.l2_pos);
    replace_non_finite(leg_state.r1_vel);
    replace_non_finite(leg_state.l1_vel);
    replace_non_finite(leg_state.r2_vel);
    replace_non_finite(leg_state.l2_vel);
    replace_non_finite(leg_state.r1_grf);
    replace_non_finite(leg_state.l1_grf);
    replace_non_finite(leg_state.r2_grf);
    replace_non_finite(leg_state.l2_grf);
    robot_state_received_.store(true, std::memory_order_release);
}

void LCMExchanger::servoStateHandler(const lcm::ReceiveBuffer*,
                                     const std::string&,
                                     const mors_msgs::servo_state_msg* msg)
{
    lock_guard<mutex> lock(servo_state_mutex);
    for (int i = 0; i < 12; ++i) {
        servo_state.position(i) = msg->position[i];
        servo_state.velocity(i) = msg->velocity[i];
        servo_state.torque(i) = msg->torque[i];
    }
    replace_non_finite(servo_state.position);
    replace_non_finite(servo_state.velocity);
    replace_non_finite(servo_state.torque);
    servo_state_received_.store(true, std::memory_order_release);
}

void LCMExchanger::gaitParamsHandler(const lcm::ReceiveBuffer*,
                                     const std::string&,
                                     const mors_msgs::gait_params_msg* msg)
{
    lock_guard<mutex> lock(gait_params_mutex);
    t_sw = msg->t_sw;
    t_st = msg->t_st;
    standing = msg->standing;
    stride_height = msg->stride_height;
    gait_type.assign(msg->gait_type, msg->gait_type + NUM_LEGS);
}

void LCMExchanger::enableHandler(const lcm::ReceiveBuffer*,
                                 const std::string&,
                                 const mors_msgs::enable_msg* msg)
{
    lock_guard<mutex> lock(enable_mutex);
    locomotion_enable = msg->locomotion_en;
    leg_controller_enable = msg->leg_controller_en;
    leg_controller_reset = msg->leg_controller_reset;
}

void LCMExchanger::robotCmdThread()
{
    robot_cmd_subscriber.subscribe(robot_cmd_channel, &LCMExchanger::robotCmdHandler, this);
    while (true) {
        robot_cmd_subscriber.handle();
    }
}

void LCMExchanger::robotStateThread()
{
    robot_state_subscriber.subscribe(robot_state_channel, &LCMExchanger::robotStateHandler, this);
    while (true) {
        robot_state_subscriber.handle();
    }
}

void LCMExchanger::servoStateThread()
{
    servo_state_subscriber.subscribe(servo_state_channel, &LCMExchanger::servoStateHandler, this);
    while (true) {
        servo_state_subscriber.handle();
    }
}

void LCMExchanger::gaitParamsThread()
{
    gait_params_subscriber.subscribe(gait_params_channel, &LCMExchanger::gaitParamsHandler, this);
    while (true) {
        gait_params_subscriber.handle();
    }
}

void LCMExchanger::enableThread()
{
    enable_subscriber.subscribe(enable_channel, &LCMExchanger::enableHandler, this);
    while (true) {
        enable_subscriber.handle();
    }
}

RobotData LCMExchanger::getRobotCmd()
{
    lock_guard<mutex> lock(robot_cmd_mutex);
    return robot_cmd;
}

RobotData LCMExchanger::getBodyState()
{
    lock_guard<mutex> lock(robot_state_mutex);
    return robot_state;
}

LegData LCMExchanger::getLegState()
{
    lock_guard<mutex> lock(robot_state_mutex);
    return leg_state;
}

ServoStateData LCMExchanger::getServoState()
{
    lock_guard<mutex> lock(servo_state_mutex);
    return servo_state;
}

void LCMExchanger::get_observation_data(RobotData& body_state,
                                          LegData& leg_state,
                                          ServoStateData& servo_state)
{
    {
        lock_guard<mutex> lock(robot_state_mutex);
        body_state = robot_state;
        leg_state = this->leg_state;
    }

    {
        lock_guard<mutex> lock(servo_state_mutex);
        servo_state = this->servo_state;
    }
}

void LCMExchanger::getWbicObservationStatus(bool& robot_state_received,
                                            bool& servo_state_received) const
{
    robot_state_received = robot_state_received_.load(std::memory_order_acquire);
    servo_state_received = servo_state_received_.load(std::memory_order_acquire);
}

void LCMExchanger::get_gait_params(double& t_st,
                                   double& t_sw,
                                   vector<double>& gait_type,
                                   bool& standing,
                                   double& stride_height)
{
    lock_guard<mutex> lock(gait_params_mutex);
    t_st = this->t_st;
    t_sw = this->t_sw;
    gait_type = this->gait_type;
    standing = this->standing;
    stride_height = this->stride_height;
}

bool LCMExchanger::get_enable()
{
    lock_guard<mutex> lock(enable_mutex);
    return locomotion_enable;
}

bool LCMExchanger::get_leg_enable()
{
    lock_guard<mutex> lock(enable_mutex);
    return leg_controller_enable;
}

void LCMExchanger::get_active_legs(vector<bool>& active_legs)
{
    lock_guard<mutex> lock(robot_cmd_mutex);
    active_legs = this->active_legs;
}

int LCMExchanger::get_adaptation_type()
{
    lock_guard<mutex> lock(robot_cmd_mutex);
    return adaptation_type;
}

void LCMExchanger::get_enable_state(bool& locomotion_enable,
                                    bool& leg_controller_enable)
{
    lock_guard<mutex> lock(enable_mutex);
    locomotion_enable = this->locomotion_enable;
    leg_controller_enable = this->leg_controller_enable;
    // leg_controller_reset = this->leg_controller_reset;
}

void LCMExchanger::sendWbcCmd(RobotData& robot_data, LegData& leg_data)
{
    const Eigen::Quaterniond q_ref =
        Eigen::AngleAxisd(robot_data.orientation[Z], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(robot_data.orientation[Y], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(robot_data.orientation[X], Eigen::Vector3d::UnitX());

    wbcCmdMsg.body.orientation_quaternion[0] = q_ref.x();
    wbcCmdMsg.body.orientation_quaternion[1] = q_ref.y();
    wbcCmdMsg.body.orientation_quaternion[2] = q_ref.z();
    wbcCmdMsg.body.orientation_quaternion[3] = q_ref.w();

    for (int i = 0; i < 3; ++i) {
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
    for (int i = 0; i < NUM_LEGS; ++i) {
        phaseSigMsg.phase[i] = phase[i];
        phaseSigMsg.phi[i] = phi[i];
    }
    phaseSigMsg.t = t;
    phase_sig_publisher.publish(phase_signal_channel, &phaseSigMsg);
}

void LCMExchanger::sendMpcCmd(RobotData& robot_cmd, vector<bool>& active_legs)
{
    for (int i = 0; i < 3; ++i) {
        mpcCmdMsg.cmd_vel[i] = robot_cmd.lin_vel[i];
        mpcCmdMsg.cmd_vel[i + 3] = robot_cmd.ang_vel[i];
        mpcCmdMsg.cmd_pose[i] = robot_cmd.pos[i];
        mpcCmdMsg.cmd_pose[i + 3] = robot_cmd.orientation[i];
        mpcCmdMsg.active_legs[i] = active_legs[i];
    }
    mpcCmdMsg.active_legs[3] = active_legs[3];

    robot_ref_publisher.publish(mpc_cmd_channel, &mpcCmdMsg);
}

void LCMExchanger::sendServoCmd(const JointVector12d& position,
                                const JointVector12d& velocity,
                                const JointVector12d& torque,
                                const JointVector12d& kp,
                                const JointVector12d& kd)
{
    for (int i = 0; i < 12; ++i) {
        servoCmdMsg.position[i] = static_cast<float>(position(i));
        servoCmdMsg.velocity[i] = static_cast<float>(velocity(i));
        servoCmdMsg.torque[i] = static_cast<float>(torque(i));
        servoCmdMsg.kp[i] = static_cast<float>(kp(i));
        servoCmdMsg.kd[i] = static_cast<float>(kd(i));
    }

    servo_cmd_publisher.publish(servo_cmd_channel, &servoCmdMsg);
}

void LCMExchanger::sendWbcState(const JointVector12d& grf)
{
    for (int i = 0; i < 3; ++i) {
        wbcStateMsg.r1_grf[i] = grf(PIN_R1 * 3 + i);
        wbcStateMsg.l1_grf[i] = grf(PIN_L1 * 3 + i);
        wbcStateMsg.r2_grf[i] = grf(PIN_R2 * 3 + i);
        wbcStateMsg.l2_grf[i] = grf(PIN_L2 * 3 + i);
    }

    wbc_state_publisher.publish(wbc_state_channel, &wbcStateMsg);
}

LCMExchanger::~LCMExchanger()
{
    if (!initialized) {
        return;
    }

    JointVector12d zero;
    zero.setZero();
    sendServoCmd(zero, zero, zero, zero, zero);
}
