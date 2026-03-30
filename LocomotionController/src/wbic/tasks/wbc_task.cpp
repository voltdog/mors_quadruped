#include "wbic/tasks/wbc_task.hpp"

WBCTask::WBCTask()
{
    Jt.setZero();
    Jtdqd.setZero();
    pos_err.setZero();
    vel_err.setZero();
    vel_des.setZero();
    acc_des.setZero();
    kp.setIdentity();
    kd.setIdentity();
}

void WBCTask::setKp(const TaskMetricMatrix& kp_)
{
    kp = kp_;
}

void WBCTask::setKd(const TaskMetricMatrix& kd_)
{
    kd = kd_;
}
