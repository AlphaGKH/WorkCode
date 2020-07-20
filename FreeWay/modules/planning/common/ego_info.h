#pragma once

#include "spider/common/macros.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/box2d.h"
#include "modules/common/proto/vehicle_config.pb.h"
#include "modules/common/proto/vehicle_state.pb.h"

namespace dharma {

namespace planning {

class EgoInfo {
public:
  ~EgoInfo() = default;

public:
  bool Update(const common::VehicleState &vehicle_state);

  common::math::Box2d ego_box() const { return ego_box_; }

private:
  void CalculateEgoBox(const common::VehicleState &vehicle_state);

private:
  common::math::Box2d ego_box_;

  common::VehicleConfig ego_vehicle_config_;

  DECLARE_SINGLETON(EgoInfo)
};

} // namespace planning

} // namespace dharma
