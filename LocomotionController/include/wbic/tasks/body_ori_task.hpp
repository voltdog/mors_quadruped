#pragma once

#include "wbic/tasks/wbc_task.hpp"

class BodyOriTask : public WBCTask
{
public:
    BodyOriTask();

    void update(
        Robot& model,
        const ConfigVector& ref_x_wcs,
        const GenCoordVector& ref_xdot_wcs,
        const GenCoordVector& ref_xddot_wcs) override;
};
