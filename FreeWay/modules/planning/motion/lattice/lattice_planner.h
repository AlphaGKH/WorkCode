#pragma once

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/planning/frame/frame.h"
#include "modules/planning/trajectory/publishable_trajectory.h"
#include "modules/planning/reference_line/reference_line_info.h"

namespace dharma {

namespace planning {

class LatticePlanner
{
public:
    LatticePlanner() = default;

    virtual ~LatticePlanner() = default;

public:
    bool Init();

    bool Plan(const common::TrajectoryPoint& planning_init_point, Frame* frame,
              ADCTrajectory* ptr_computed_trajectory);

private:
    bool PlanOnReferenceLine(const common::TrajectoryPoint& planning_init_point, Frame* frame,
                             ReferenceLineInfo* reference_line_info);


};



}

}
