#include "planning/motion/fp_path_paln/fp_path_planner.h"

namespace planning {

bool FPPathPlanner::Init(const FpPathPlanConfig& config, const perception::OgmConfig& ogm_config){
    parallel_graph_ = std::make_unique<ParallelGraph>(config, ogm_config);

    is_inited_ = true;

    return is_inited_;
}

bool FPPathPlanner::Process(const common::TrajectoryPoint& init_point,
                            const common::VehicleState& vehicle_state,
                            const ReferenceLine& reference_line,
                            const perception::GridMap& grid_map,
                            PathData* min_cost_path){
    if(!is_inited_){
        AERROR << "FPPathPlanner is not inited!";
        return false;
    }

    if(!parallel_graph_->FindPathTunnel(init_point, vehicle_state, reference_line, grid_map, min_cost_path)){
        AWARN << "Find min_cost_path failed based on ( "
              << "init_point: " << init_point.DebugString()
              << "vehicle_state: " << vehicle_state.DebugString()
              << "reference_line: " << reference_line.DebugString()
              << " )";
        return false;
    }

    return true;
}



}
