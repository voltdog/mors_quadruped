#pragma once

#include "wbic/tasks/wbc_task.hpp"

class BodyPosTask : public WBCTask
{
public:
    BodyPosTask();

    void update(
        Robot& model,
        const ConfigVector& ref_x_wcs,
        const GenCoordVector& ref_xdot_wcs,
        const GenCoordVector& ref_xddot_wcs) override;
};
