#include "modules/planning/common/util/util.h"

#include <cmath>

namespace dharma {

namespace planning {

namespace util {

bool IsVehicleStateValid(const common::VehicleState& vehicle_state) {
    if (std::isnan(vehicle_state.x()) || std::isnan(vehicle_state.y()) ||
            std::isnan(vehicle_state.z()) || std::isnan(vehicle_state.heading()) ||
            std::isnan(vehicle_state.kappa()) ||
            std::isnan(vehicle_state.linear_velocity()) ||
            std::isnan(vehicle_state.linear_acceleration())) {
        return false;
    }
    return true;
}

}

}

}
