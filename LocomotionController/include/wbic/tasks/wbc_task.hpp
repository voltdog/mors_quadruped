#pragma once

#include "Robot.hpp"
#include "data_types.hpp"

class WBCTask
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static constexpr int n_leg = mors::wbic::kNumLegs;
    static constexpr int nv = mors::wbic::kNv;
    static constexpr int nq = mors::wbic::kNq;
    static constexpr int task_dim = mors::wbic::kTaskDim;

    using ConfigVector = mors::wbic::Vector19d;
    using GenCoordVector = mors::wbic::Vector18d;
    using TaskVector = mors::wbic::Vector3d;
    using TaskJacobianMatrix = mors::wbic::Matrix3_18d;
    using TaskMetricMatrix = mors::wbic::Matrix3d;

protected:
    TaskJacobianMatrix Jt;
    TaskVector Jtdqd;
    TaskVector pos_err;
    TaskVector vel_err;
    TaskVector vel_des;
    TaskVector acc_des;
    TaskMetricMatrix kp;
    TaskMetricMatrix kd;

public:
    WBCTask();
    virtual ~WBCTask() = default;

    virtual void update(
        Robot& model,
        const ConfigVector& ref_x_wcs,
        const GenCoordVector& ref_xdot_wcs,
        const GenCoordVector& ref_xddot_wcs) = 0;

    const TaskJacobianMatrix& getJacobian() const { return Jt; }
    const TaskVector& getJdqd() const { return Jtdqd; }

    const TaskVector& getPosErr() const { return pos_err; }
    const TaskVector& getVelDes() const { return vel_des; }
    const TaskVector& getAccDes() const { return acc_des; }

    void setKp(const TaskMetricMatrix& kp_);
    void setKd(const TaskMetricMatrix& kd_);
};
