#pragma once

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/proto/vehicle_state.pb.h"

namespace dharma {
namespace common {

class VehicleModel {
public:
  VehicleModel() = delete;

  static VehicleState Predict(const double predicted_time_horizon,
                              const VehicleState &cur_vehicle_state);

private:
  static void
  RearCenteredKinematicBicycleModel(const double predicted_time_horizon,
                                    const VehicleState &cur_vehicle_state,
                                    VehicleState *predicted_vehicle_state);
};

} // namespace common
} // namespace dharma
