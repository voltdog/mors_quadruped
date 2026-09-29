#include "wbic/tasks/tip_pos_task.hpp"

TipPosTask::TipPosTask(int leg) : WBCTask(), leg_id(leg)
{
    kp = 100.0 * TaskMetricMatrix::Identity();
    kd = 5.0 * TaskMetricMatrix::Identity();
}

void TipPosTask::update(
    Robot& model,
    const ConfigVector& ref_x_wcs,
    const GenCoordVector& ref_xdot_wcs,
    const GenCoordVector& ref_xddot_wcs)
{
    Jt = model.get_leg_pos_jacobian(leg_id);
    Jtdqd = model.get_leg_pos_jdqd(leg_id);

    const TaskVector act_pos = model.get_tip_pos_global(leg_id);
    const TaskVector act_vel = model.get_tip_vel_global(leg_id);

    const TaskVector ref_pos = ref_x_wcs.segment<3>(7 + leg_id * 3);
    const TaskVector ref_vel = ref_xdot_wcs.segment<3>(6 + leg_id * 3);
    const TaskVector ref_acc = ref_xddot_wcs.segment<3>(6 + leg_id * 3);

    pos_err = ref_pos - act_pos;
    vel_err = ref_vel - act_vel;
    vel_des = ref_vel;
    acc_des = ref_acc + kp * pos_err + kd * vel_err;
}
