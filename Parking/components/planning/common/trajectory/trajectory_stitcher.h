#ifndef PLANNING_COMMON_TRAJECTORY_TRAJECTORY_STITCHER_H_
#define PLANNING_COMMON_TRAJECTORY_TRAJECTORY_STITCHER_H_

#include "planning/common/trajectory/publishable_trajectory.h"
#include "common/proto/vehicle_state.pb.h"
#include "planning/proto/planning_config.pb.h"

namespace planning {

class TrajectoryStitcher
{
public:
    TrajectoryStitcher() = delete;

    static void TransformLastPublishedTrajectory(const double x_diff,const double y_diff,
                                                 const double theta_diff,
                                                 PublishableTrajectory* prev_trajectory);

    static std::vector<common::TrajectoryPoint> ComputeStitchingTrajectory(
            const common::VehicleState& vehicle_state, const double current_timestamp,
            const double planning_cycle_time, const PublishableTrajectory* prev_trajectory,
            const TrajectoryStitchingConfig& config,
            bool* is_replan);

private:
    static std::pair<double, double> ComputePositionProjection(const double x,const double y,
                                                               const common::TrajectoryPoint& matched_trajectory_point);

    static std::vector<common::TrajectoryPoint> ComputeReinitStitchingTrajectory(
            const common::VehicleState& vehicle_state);

};

}


#endif
