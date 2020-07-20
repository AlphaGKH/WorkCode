#include "modules/planning/reference_line/reference_line_info.h"

#include "modules/common/configs/vehicle_config_helper.h"

#include "modules/planning/common/planning_gflags.h"

namespace dharma {

namespace planning {

ReferenceLineInfo::ReferenceLineInfo(
    const common::VehicleState &vehicle_state,
    const common::TrajectoryPoint &adc_planning_point,
    const ReferenceLine &reference_line)
    : vehicle_state_(vehicle_state), adc_planning_point_(adc_planning_point),
      reference_line_(reference_line) {}

bool ReferenceLineInfo::Init(const std::vector<const Obstacle *> &obstacles) {
  const auto &param = common::VehicleConfigHelper::GetConfig().vehicle_param();
  // stitching point
  const auto &path_point = adc_planning_point_.path_point();
  common::math::Vec2d position(path_point.x(), path_point.y());
  common::math::Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);

  common::math::Vec2d center(position +
                             vec_to_center.rotate(path_point.theta()));
  common::math::Box2d box(center, path_point.theta(), param.length(),
                          param.width());
  // realtime vehicle position
  common::math::Vec2d vehicle_position(vehicle_state_.x(), vehicle_state_.y());
  common::math::Vec2d vehicle_center(
      vehicle_position + vec_to_center.rotate(vehicle_state_.heading()));
  common::math::Box2d vehicle_box(vehicle_center, vehicle_state_.heading(),
                                  param.length(), param.width());

  if (!reference_line_.GetSLBoundary(box, &adc_sl_boundary_)) {
    AERROR << "Failed to get ADC boundary from box: " << box.DebugString();
    return false;
  }

  if (adc_sl_boundary_.end_s() < 0 ||
      adc_sl_boundary_.start_s() > reference_line_.Length()) {
    AWARN << "Vehicle SL " << adc_sl_boundary_.ShortDebugString()
          << " is not on reference line:[0, " << reference_line_.Length()
          << "]";
  }

  static constexpr double kOutOfReferenceLineL = 10.0; // in meters
  if (adc_sl_boundary_.start_l() > kOutOfReferenceLineL ||
      adc_sl_boundary_.end_l() < -kOutOfReferenceLineL) {
    AERROR << "Ego vehicle is too far away from reference line.";
    return false;
  }

  // set lattice planning target speed limit;
  SetLatticeCruiseSpeed(FLAGS_default_cruise_speed);
  return true;
}

void ReferenceLineInfo::SetTrajectory(const DiscretizedTrajectory &trajectory) {
  discretized_trajectory_ = trajectory;
}

} // namespace planning

} // namespace dharma
