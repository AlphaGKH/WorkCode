#ifndef PLANNING_MOTION_FP_PATH_PLAN_FP_PATH_PLANNER_H_
#define PLANNING_MOTION_FP_PATH_PLAN_FP_PATH_PLANNER_H_

#include <memory>

#include "common/proto/pnc_point.pb.h"
#include "common/proto/vehicle_state.pb.h"

#include "perception/GridMap.hpp"

#include "planning/common/path/path_data.h"
#include "planning/reference_line/reference_line.h"
#include "planning/motion/fp_path_paln/parallel_graph.h"

namespace planning {

class FPPathPlanner
{
public:
    FPPathPlanner() = default;
    ~FPPathPlanner() = default;

public:
    bool Init(const FpPathPlanConfig& config, const perception::OgmConfig& ogm_config);

    bool Process(const common::TrajectoryPoint& init_point,
                 const common::VehicleState& vehicle_state,
                 const ReferenceLine& reference_line,
                 const perception::GridMap& grid_map,
                 PathData* min_cost_path);

private:
    std::unique_ptr<ParallelGraph> parallel_graph_;

private:
    bool is_inited_ = false;

};

}


#endif
