#pragma once

#include "wbic/tasks/wbc_task.hpp"

class TipPosTask : public WBCTask
{
public:
    explicit TipPosTask(int leg);

    void update(
        Robot& model,
        const ConfigVector& ref_x_wcs,
        const GenCoordVector& ref_xdot_wcs,
        const GenCoordVector& ref_xddot_wcs) override;

private:
    int leg_id;
};
