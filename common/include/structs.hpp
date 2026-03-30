#ifndef _structs_hpp_
#define _structs_hpp_

#include <array>
#include <cstdint>
#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;

constexpr int R1 = 0;
constexpr int L1 = 1;
constexpr int R2 = 2;
constexpr int L2 = 3;

constexpr int PIN_R1 = 2;
constexpr int PIN_L1 = 0;
constexpr int PIN_R2 = 3;
constexpr int PIN_L2 = 1;
constexpr int PIN_START_IDX = 7;

constexpr int X = 0;
constexpr int Y = 1;
constexpr int Z = 2;

static constexpr int INCL_ADAPT = 0;
static constexpr int HEIGHT_ADAPT = 1;
static constexpr int NOADAPT = 2;

static constexpr int SWING = 0;
static constexpr int STANCE = 1;
static constexpr int LATE_CONTACT = 2;
static constexpr int EARLY_CONTACT = 3;

static constexpr int NUM_LEGS = 4;

static constexpr int LEG_CONTROL   = 1;
static constexpr int SERVO_CONTROL = 2;

using JointVector12d = Matrix<double, 12, 1>;
using PhaseVector4i = Matrix<int, NUM_LEGS, 1>;

struct RobotData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RobotData()
    {
        pos.setZero();
        lin_vel.setZero();
        orientation.setZero();
        orientation_quaternion << 0.0, 0.0, 0.0, 1.0;
        ang_vel.setZero();
    }

    Vector3d pos;
    Vector3d lin_vel;
    Vector3d orientation;
    Vector4d orientation_quaternion;
    Vector3d ang_vel;
};

struct LegData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LegData()
        : contacts(NUM_LEGS, false)
    {
        r1_grf.setZero();
        l1_grf.setZero();
        r2_grf.setZero();
        l2_grf.setZero();

        r1_pos.setZero();
        l1_pos.setZero();
        r2_pos.setZero();
        l2_pos.setZero();

        r1_vel.setZero();
        l1_vel.setZero();
        r2_vel.setZero();
        l2_vel.setZero();

        r1_acc.setZero();
        l1_acc.setZero();
        r2_acc.setZero();
        l2_acc.setZero();

        r1_kp.setZero();
        l1_kp.setZero();
        r2_kp.setZero();
        l2_kp.setZero();

        r1_kd.setZero();
        l1_kd.setZero();
        r2_kd.setZero();
        l2_kd.setZero();
    }

    Vector3d r1_grf;
    Vector3d l1_grf;
    Vector3d r2_grf;
    Vector3d l2_grf;

    Vector3d r1_pos;
    Vector3d l1_pos;
    Vector3d r2_pos; 
    Vector3d l2_pos;

    Vector3d r1_vel; 
    Vector3d l1_vel; 
    Vector3d r2_vel; 
    Vector3d l2_vel;

    Vector3d r1_acc; 
    Vector3d l1_acc; 
    Vector3d r2_acc; 
    Vector3d l2_acc;

    vector<bool> contacts;

    Vector3d r1_kp; 
    Vector3d l1_kp; 
    Vector3d r2_kp; 
    Vector3d l2_kp; 

    Vector3d r1_kd; 
    Vector3d l1_kd; 
    Vector3d r2_kd; 
    Vector3d l2_kd; 
};

struct RobotPhysicalParams
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double M_b;
    MatrixXd I_b;
    double bx, by;

    double m1, m2, m3;
    double l1, l2, l3;
    double d1, d2, d3;
    double l_cx_3, l_cz_2;
    
    double g;

    double joint_tau_max_array[3];
    double joint_vel_max_array[3];
    double kt;
    double gear_ratio;
};

struct ImuData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuData()
    {
        orientation_euler.setZero();
        orientation_quaternion << 0.0, 0.0, 0.0, 1.0;
        ang_vel.setZero();
        lin_accel.setZero();
    }

    Vector3d orientation_euler;
    Vector4d orientation_quaternion;
    Vector3d ang_vel;
    Vector3d lin_accel;
};

struct ServoData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VectorXd pos;
    VectorXd vel;
    VectorXd torq;
    VectorXd kp;
    VectorXd kd;
};

struct Odometry
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Odometry()
    {
        position.setZero();
        orientation_euler.setZero();
        orientation_quaternion << 0.0, 0.0, 0.0, 1.0;
        lin_vel.setZero();
        ang_vel.setZero();
    }

    Vector3d position;
    Vector3d orientation_euler;
    Vector4d orientation_quaternion;
    Vector3d lin_vel;
    Vector3d ang_vel;
};

struct ServoStateData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ServoStateData()
    {
        position.setZero();
        velocity.setZero();
        torque.setZero();
    }

    JointVector12d position;
    JointVector12d velocity;
    JointVector12d torque;
    // std::uint64_t sequence = 0;
};

struct WbcDesiredCommand
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    WbcDesiredCommand()
    {
        phase_signal.fill(STANCE);
        active_legs.fill(true);
    }

    RobotData body_cmd;
    LegData leg_cmd;
    std::array<int, NUM_LEGS> phase_signal{STANCE, STANCE, STANCE, STANCE};
    std::array<bool, NUM_LEGS> active_legs{true, true, true, true};
    bool locomotion_enabled = false;
    // std::uint64_t sequence = 0;
};

struct WbcOutputData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    WbcOutputData()
    {
        joint_pos.setZero();
        joint_vel.setZero();
        joint_torque.setZero();
        motor_kp.setZero();
        motor_kd.setZero();
        grf.setZero();
    }

    JointVector12d joint_pos;
    JointVector12d joint_vel;
    JointVector12d joint_torque;
    JointVector12d motor_kp;
    JointVector12d motor_kd;
    JointVector12d grf;
    bool valid = false;
    // std::uint64_t sequence = 0;
};



#endif //_structs_hpp_
