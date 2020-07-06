#ifndef PLANNING_MOTION_POLY_SPEED_PLAN_POLY_SPEED_PLANNER_H_
#define PLANNING_MOTION_POLY_SPEED_PLAN_POLY_SPEED_PLANNER_H_

#include "common/proto/pnc_point.pb.h"
#include "planning/common/path/path_data.h"
#include "planning/common/speed/speed_data.h"
#include "planning/motion/poly_speed_plan/poly_st_graph.h"

namespace planning {

class PolySpeedPlanner
{
public:
    PolySpeedPlanner() = default;
    ~PolySpeedPlanner() = default;

public:
    bool Init(const PolySpeedPlanConfig& config);
    bool Process(const common::TrajectoryPoint& init_point,
                 const PathData& path_data,
                 SpeedData* speed_data);


private:
    std::unique_ptr<PolyStGraph> poly_st_graph_;


private:
    bool is_inited_ = false;
};

}



#endif
