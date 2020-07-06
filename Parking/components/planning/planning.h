#ifndef PLANNING_PLANNING_H_
#define PLANNING_PLANNING_H_

#include "common/proto/vehicle_state.pb.h"
#include "common/gflags_common.h"

#include "map/road_map/road_map.h"

#include "perception/GridMap.hpp"

#include "planning/common/trajectory/publishable_trajectory.h"
#include "planning/reference_line/reference_line.h"
#include "planning/reference_line/reference_line_provider.h"
#include "planning/motion/fp_path_paln/fp_path_planner.h"
#include "planning/motion/poly_speed_plan/poly_speed_planner.h"

#include "planning/proto/planning_config.pb.h"

namespace planning {

class Planning
{
public:
    Planning() = default;
    ~Planning();

    bool Start();

    void Stop();

private:
    bool Init();

    void PlanningThread();

    void RunOnce();

    bool PlanProcess(const common::VehicleState& vehicle_state,
                     const ReferenceLine& reference_line,
                     const perception::GridMap& grid_map);

    bool CombinePathAndSpeedProfile(const PathData& path_data, const SpeedData& speed_data,
                                    const common::TrajectoryPoint& planning_start_point,
                                    DiscretizedTrajectory* discretized_trajectory);


private:
    bool is_stop_ = false;

    double last_planning_time_ = 0;

    std::unique_ptr<std::thread> planning_thread_ptr_;

private:
    std::unique_ptr<map::RoadMap> road_map_ptr_;
    std::unique_ptr<ReferenceLineProvider> reference_line_provider_;

    std::unique_ptr<FPPathPlanner> path_planner_;
    std::unique_ptr<PolySpeedPlanner> speed_planner_;


private:
    // for trajectory stitcher
    std::unique_ptr<PublishableTrajectory> last_publishable_trajectory_;

private:
    PlanningConfig planning_config_;

};



}


#endif
