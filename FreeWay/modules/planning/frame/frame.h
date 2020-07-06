#pragma once
#include <list>

#include "modules/planning/reference_line/reference_line_info.h"
#include "modules/planning/obstacle/obstacle.h"

namespace dharma {

namespace planning {

class Frame {
public:
    Frame(uint32_t sequence_num, const std::shared_ptr<canbus::Chassis>& chassis,
          const std::shared_ptr<prediction::PredictionObstacles>& prediction_obstacles,
          const common::TrajectoryPoint& planning_start_point);

    virtual ~Frame() = default;

public:
    std::list<ReferenceLineInfo> *mutable_reference_line_info();
    common::TrajectoryPoint planning_start_point_;

    const std::vector<const Obstacle *> obstacles() const;

private:
    uint32_t sequence_num_ = 0;


private:
    std::list<ReferenceLineInfo> reference_line_info_;

    ThreadSafeIndexedObstacles obstacles_;

};


}
}
