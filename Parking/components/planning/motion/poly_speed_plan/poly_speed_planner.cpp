#include "planning/motion/poly_speed_plan/poly_speed_planner.h"

namespace planning {

bool PolySpeedPlanner::Init(const PolySpeedPlanConfig& config){
    poly_st_graph_ = std::make_unique<PolyStGraph>(config);

    is_inited_ = true;
    return is_inited_;
}

bool PolySpeedPlanner::Process(const common::TrajectoryPoint &init_point,
                               const PathData &path_data, SpeedData *speed_data){
    if(!is_inited_){
        return false;
    }

    if(path_data.discretized_path().size() == 0){
        AERROR << "path_data is empty";
        return false;
    }

    if(!poly_st_graph_->FindStTunnel(init_point, path_data, speed_data)){
        AWARN << "Find speed_data failed based on ( "
              << "init_point: " << init_point.DebugString();
        return false;
    }

    return true;
}

}
