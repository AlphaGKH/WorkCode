#pragma once

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

  void SetLatticeCruiseSpeed(double speed) {
    planning_target_.set_cruise_speed(speed);
  }

  const PlanningTarget &planning_target() const { return planning_target_; }

  const ReferenceLine &reference_line() const { return reference_line_; }

public:
  void SetTrajectory(const DiscretizedTrajectory &trajectory) {
    discretized_trajectory_ = trajectory;
  };
  const DiscretizedTrajectory &trajectory() const {
    return discretized_trajectory_;
  }

  void AddCost(double cost) { cost_ += cost; }
  void SetCost(double cost) { cost_ = cost; }
  double Cost() const { return cost_; }

  void SetPriorityCost(double cost) { priority_cost_ = cost; }
  double PriorityCost() const { return priority_cost_; }

  /**
   * Set if the vehicle can drive following this reference line
   * A planner need to set this value to true if the reference line is OK
   */
  void SetDrivable(bool drivable) { is_drivable_ = drivable; }
  bool IsDrivable() const { return is_drivable_; }

private:
  double priority_cost_ = 0.0;

  PlanningTarget planning_target_;

private:
  DiscretizedTrajectory discretized_trajectory_;

  double cost_ = 0.0;

  bool is_drivable_ = true;

private:
  const common::VehicleState vehicle_state_;

  const common::TrajectoryPoint adc_planning_point_;

  ReferenceLine reference_line_;

private:
  /**
   * @brief SL boundary of stitching point (starting point of plan trajectory)
   * relative to the reference line
   */
  SLBoundary adc_sl_boundary_; // not used 2020-07-20
};

} // namespace planning
} // namespace dharma
