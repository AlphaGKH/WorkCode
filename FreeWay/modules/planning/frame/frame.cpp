#include "modules/planning/frame/frame.h"

namespace dharma {

namespace planning {

Frame::Frame(uint32_t sequence_num, const std::shared_ptr<canbus::Chassis>& chassis,
             const std::shared_ptr<prediction::PredictionObstacles>& prediction_obstacles,
             const common::TrajectoryPoint& planning_start_point){

}

std::list<ReferenceLineInfo> *Frame::mutable_reference_line_info() {
    return &reference_line_info_;
}

const std::vector<const Obstacle *> Frame::obstacles() const {
    return obstacles_.Items();
}


}

}
