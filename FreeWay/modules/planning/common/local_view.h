#pragma once

#include <memory>

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"

namespace dharma {

namespace planning {

struct LocalView {
  std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles;
  std::shared_ptr<canbus::Chassis> chassis;
  std::shared_ptr<localization::Localization> localization;
};

} // namespace planning

} // namespace dharma
