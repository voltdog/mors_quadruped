#include "wbic/tasks/body_ori_task.hpp"

#include <Eigen/Geometry>

BodyOriTask::BodyOriTask() : WBCTask()
{
    kp = 50.0 * TaskMetricMatrix::Identity();
    kd = TaskMetricMatrix::Identity();
}

void BodyOriTask::update(
    Robot& model,
    const ConfigVector& ref_x_wcs,
    const GenCoordVector& ref_xdot_wcs,
    const GenCoordVector& ref_xddot_wcs)
{
    Jt = model.get_body_ori_jacobian();
    Jtdqd = model.get_body_ori_jdqd();

    const Eigen::Vector4d act_ori = model.get_body_ori_global();
    const TaskVector act_omega = model.get_body_angvel_global();

    const Eigen::Vector4d ref_ori = ref_x_wcs.segment<4>(3);
    const TaskVector ref_omega = ref_xdot_wcs.segment<3>(3);
    const TaskVector ref_angacc = ref_xddot_wcs.segment<3>(3);

    const Eigen::Quaterniond q_act(act_ori[3], act_ori[0], act_ori[1], act_ori[2]);
    const Eigen::Quaterniond q_ref(ref_ori[3], ref_ori[0], ref_ori[1], ref_ori[2]);

    const TaskMetricMatrix R = q_act.toRotationMatrix();
    const TaskMetricMatrix R_ref = q_ref.toRotationMatrix();
    const TaskMetricMatrix R_diff = R_ref * R.transpose();
    const Eigen::AngleAxisd aa(R_diff);

    pos_err = aa.axis() * aa.angle();
    vel_err = ref_omega - act_omega;
    vel_des = ref_omega;
    acc_des = ref_angacc + kp * pos_err + kd * vel_err;
}
