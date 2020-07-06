#include "modules/planning/reference_line/reference_line_info.h"

namespace dharma {

namespace planning {

ReferenceLineInfo::ReferenceLineInfo(const common::VehicleState& vehicle_state,
                                     const common::TrajectoryPoint& adc_planning_point,
                                     const ReferenceLine& reference_line){

}

void ReferenceLineInfo::SetLatticeCruiseSpeed(double speed) {
    planning_target_.set_cruise_speed(speed);
}

void ReferenceLineInfo::SetTrajectory(const DiscretizedTrajectory& trajectory) {
    discretized_trajectory_ = trajectory;
}

void ReferenceLineInfo::SetDrivable(bool drivable) { is_drivable_ = drivable; }

}

}
