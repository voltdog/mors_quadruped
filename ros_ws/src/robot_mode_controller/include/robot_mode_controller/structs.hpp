#ifndef _structs_hpp_
#define _structs_hpp_

#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace Eigen;

constexpr int R1 = 0;
constexpr int L1 = 1;
constexpr int R2 = 2;
constexpr int L2 = 3;

constexpr int X = 0;
constexpr int Y = 1;
constexpr int Z = 2;

constexpr int ROLL = 0;
constexpr int PITCH = 1;
constexpr int YAW = 2;

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

struct RobotData
{
    VectorXd pos;
    VectorXd lin_vel;
    VectorXd orientation;
    VectorXd ang_vel;
};

struct LegData
{
    VectorXd r1_grf;
    VectorXd l1_grf;
    VectorXd r2_grf;
    VectorXd l2_grf;

    VectorXd r1_pos;
    VectorXd l1_pos;
    VectorXd r2_pos; 
    VectorXd l2_pos;

    VectorXd r1_vel; 
    VectorXd l1_vel; 
    VectorXd r2_vel; 
    VectorXd l2_vel;

    VectorXd r1_acc; 
    VectorXd l1_acc; 
    VectorXd r2_acc; 
    VectorXd l2_acc;

    vector<bool> contacts;

    VectorXd r1_kp; 
    VectorXd l1_kp; 
    VectorXd r2_kp; 
    VectorXd l2_kp; 

    VectorXd r1_kd; 
    VectorXd l1_kd; 
    VectorXd r2_kd; 
    VectorXd l2_kd; 
};

#endif //_structs_hpp_