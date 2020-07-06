#pragma once

#include "modules/planning/proto/lattice_structure.pb.h"
#include "modules/planning/reference_line/reference_line.h"
#include "modules/planning/trajectory/discretized_trajectory.h"

#include "modules/common/proto/vehicle_state.pb.h"

namespace dharma {

namespace planning {

class ReferenceLineInfo
{
public:
    ReferenceLineInfo() = default;

    explicit ReferenceLineInfo(const common::VehicleState& vehicle_state,
                               const common::TrajectoryPoint& adc_planning_point,
                               const ReferenceLine& reference_line);

public:
    const ReferenceLine& reference_line() const {
        return reference_line_;
    }

    const PlanningTarget& planning_target() const {
        return planning_target_; }

    double PriorityCost() const { return priority_cost_; }

    void AddCost(double cost) { cost_ += cost; }

public:
    void SetPriorityCost(double cost) {
        priority_cost_ = cost;
    }

    void SetLatticeCruiseSpeed(double speed);

    void SetTrajectory(const DiscretizedTrajectory& trajectory);

    void SetCost(double cost) { cost_ = cost; }

    void SetDrivable(bool drivable);

private:
    double priority_cost_ = 0.0;

    ReferenceLine reference_line_;

    PlanningTarget planning_target_;

    DiscretizedTrajectory discretized_trajectory_;

    double cost_ = 0.0;

    bool is_drivable_ = true;
};



}
}
