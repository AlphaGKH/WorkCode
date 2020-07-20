#pragma once

#include "modules/common/proto/vehicle_state.pb.h"
#include "modules/routing/proto/routing.pb.h"

namespace dharma {

namespace planning {

namespace util {

bool IsVehicleStateValid(const common::VehicleState& vehicle_state);

bool IsDifferentRouting(const routing::RoutingResponse& first,
                        const routing::RoutingResponse& second);

}

}

}
