#pragma once

#include "modules/common/proto/vehicle_state.pb.h"

namespace dharma {

namespace planning {

namespace util {

bool IsVehicleStateValid(const common::VehicleState &vehicle_state);
}

} // namespace planning

} // namespace dharma
