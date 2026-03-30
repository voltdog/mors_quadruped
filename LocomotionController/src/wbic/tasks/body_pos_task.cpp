#include "wbic/tasks/body_pos_task.hpp"

BodyPosTask::BodyPosTask() : WBCTask()
{
    kp = 50.0 * TaskMetricMatrix::Identity();
    kd = TaskMetricMatrix::Identity();
}

void BodyPosTask::update(
    Robot& model,
    const ConfigVector& ref_x_wcs,
    const GenCoordVector& ref_xdot_wcs,
    const GenCoordVector& ref_xddot_wcs)
{
    Jt = model.get_body_pos_jacobian();
    Jtdqd = model.get_body_pos_jdqd();

    const TaskVector act_pos = model.get_body_pos_global();
    const TaskVector act_vel = model.get_body_vel_global();
    const TaskVector ref_pos = ref_x_wcs.segment<3>(0);
    const TaskVector ref_vel = ref_xdot_wcs.segment<3>(0);
    const TaskVector ref_acc = ref_xddot_wcs.segment<3>(0);

    pos_err = ref_pos - act_pos;
    vel_err = ref_vel - act_vel;
    vel_des = ref_vel;
    acc_des = ref_acc + kp * pos_err + kd * vel_err;
}
