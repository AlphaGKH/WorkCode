#pragma once

#include "spider/common/macros.h"

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/proto/vehicle_state.pb.h"
#include "modules/localization/proto/localization.pb.h"

#include "modules/common/math/vec2d.h"

namespace dharma {

namespace common {

class VehicleStateProvider {

public:
  bool Update(const localization::Localization &localization,
              const canbus::Chassis &chassis);

private:
  bool
  ConstructExceptLinearVelocity(const localization::Localization &localization);

public:
  const VehicleState &vehicle_state() const { return vehicle_state_; };

  /**
   * @brief Estimate future position from current position and heading,
   *        along a period of time, by constant linear velocity,
   *        linear acceleration, angular velocity.
   * @param t The length of time period.
   * @return The estimated future position in time t.
   */
  math::Vec2d EstimateFuturePosition(const double t) const;

private:
  common::VehicleState vehicle_state_;

  DECLARE_SINGLETON(VehicleStateProvider)
};

} // namespace common

} // namespace dharma
