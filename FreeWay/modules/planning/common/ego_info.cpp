#include "modules/planning/common/ego_info.h"

#include "spider/common/log.h"

namespace dharma {

namespace planning {

EgoInfo::EgoInfo() {
  ego_vehicle_config_ = common::VehicleConfigHelper::GetConfig();
}

bool EgoInfo::Update(const common::VehicleState &vehicle_state) {
  CalculateEgoBox(vehicle_state);
  return true;
}

void EgoInfo::CalculateEgoBox(const common::VehicleState &vehicle_state) {
  const auto &param = ego_vehicle_config_.vehicle_param();
  ADEBUG << "param: " << param.DebugString();

  common::math::Vec2d vec_to_center(
      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);

  common::math::Vec2d position(vehicle_state.x(), vehicle_state.y());
  common::math::Vec2d center(position +
                             vec_to_center.rotate(vehicle_state.heading()));

  ego_box_ = common::math::Box2d(center, vehicle_state.heading(),
                                 param.length(), param.width());
}

} // namespace planning

} // namespace dharma
