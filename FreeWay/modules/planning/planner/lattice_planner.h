#pragma once

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/frame/frame.h"

namespace dharma {

namespace planning {

class LatticePlanner{
public:
    LatticePlanner() = default;

    virtual ~LatticePlanner() = default;

public:
    bool Plan(const common::TrajectoryPoint& planning_init_point, Frame* frame);


    bool PlanOnReferenceLine(const common::TrajectoryPoint &planning_init_point,
                             Frame *frame, ReferenceLineInfo *reference_line_info);

};

}

}
