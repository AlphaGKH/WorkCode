#ifndef COMMON_ADAPTERS_MESSAGE_ADAPTERS_H_
#define COMMON_ADAPTERS_MESSAGE_ADAPTERS_H_

#include "common/adapters/adapter.h"

#include "perception/Lidar2D.hpp"
#include "perception/GridMap.hpp"

#include "chassis/Chassis.hpp"

#include "control/Command.hpp"

#include "planning/LcmMapPathPoint.hpp"
#include "planning/LcmMapPath.hpp"
#include "planning/LcmReferencePoint.hpp"
#include "planning/LcmReferenceLine.hpp"
#include "planning/LcmTrajectoryPoint.hpp"
#include "planning/LcmTrajectory.hpp"

namespace common {

namespace adapter {

// perception
using Lidar2DAdapter = Adapter<perception::Lidar2D>;
using GridMapAdapter = Adapter<perception::GridMap>;

// chassis
using ChassisAdapter = Adapter<chassis::Chassis>;

// planning
using MapPathAdapter = Adapter<planning::LcmMapPath>;
using RefLineAdapter = Adapter<planning::LcmReferenceLine>;
using TrajectoryAdapter = Adapter<planning::LcmTrajectory>;

// control
using CommandAdapter = Adapter<control::Command>;

}

}



#endif
