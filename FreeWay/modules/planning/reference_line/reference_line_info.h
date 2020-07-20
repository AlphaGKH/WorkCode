#pragma once

#include "modules/map/pnc_map/route_segments.h"

#include "modules/planning/obstacle/obstacle.h"
#include "modules/planning/proto/lattice_structure.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/trajectory/discretized_trajectory.h"

#include "modules/common/proto/vehicle_state.pb.h"

namespace dharma {

namespace planning {

class ReferenceLineInfo {
public:
  ReferenceLineInfo() = default;

  explicit ReferenceLineInfo(const common::VehicleState &vehicle_state,
                             const common::TrajectoryPoint &adc_planning_point,
                             const ReferenceLine &reference_line);

public:
  bool Init(const std::vector<const Obstacle *> &obstacles);

public:
  const ReferenceLine &reference_line() const { return reference_line_; }

  const DiscretizedTrajectory &trajectory() const {
    return discretized_trajectory_;
  }

public:
  void SetPriorityCost(double cost) { priority_cost_ = cost; }

  double PriorityCost() const { return priority_cost_; }

  void SetLatticeCruiseSpeed(double speed) {
    planning_target_.set_cruise_speed(speed);
  }

  const PlanningTarget &planning_target() const { return planning_target_; }

private:
  double priority_cost_ = 0.0;

  PlanningTarget planning_target_;

public:
  void AddCost(double cost) { cost_ += cost; }

  void SetTrajectory(const DiscretizedTrajectory &trajectory);

  void SetCost(double cost) { cost_ = cost; }

  void SetDrivable(bool drivable) { is_drivable_ = drivable; }

private:
  /**
   * @brief SL boundary of stitching point (starting point of plan trajectory)
   * relative to the reference line
   */
  SLBoundary adc_sl_boundary_;

private:
  DiscretizedTrajectory discretized_trajectory_;

  double cost_ = 0.0;

  bool is_drivable_ = true;

  // input value
private:
  const common::VehicleState vehicle_state_;

  const common::TrajectoryPoint adc_planning_point_;

  ReferenceLine reference_line_;
};

} // namespace planning
} // namespace dharma
