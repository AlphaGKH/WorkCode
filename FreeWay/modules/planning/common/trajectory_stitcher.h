#pragma once

#include <string>
#include <utility>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/common/proto/vehicle_state.pb.h"

#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/trajectory/publishable_trajectory.h"

namespace dharma {
namespace planning {

class TrajectoryStitcher {
public:
  TrajectoryStitcher() = delete;

  static std::vector<common::TrajectoryPoint> ComputeStitchingTrajectory(
      const common::VehicleState &vehicle_state, const double current_timestamp,
      const double planning_cycle_time, const size_t preserved_points_num,
      const bool replan_by_offset, const PublishableTrajectory *prev_trajectory,
      std::string *replan_reason);

  static std::vector<common::TrajectoryPoint>
  ComputeReinitStitchingTrajectory(const double planning_cycle_time,
                                   const common::VehicleState &vehicle_state);

private:
  static std::pair<double, double> ComputePositionProjection(
      const double x, const double y,
      const common::TrajectoryPoint &matched_trajectory_point);

  static common::TrajectoryPoint ComputeTrajectoryPointFromVehicleState(
      const common::VehicleState &vehicle_state);
};

} // namespace planning
} // namespace dharma
