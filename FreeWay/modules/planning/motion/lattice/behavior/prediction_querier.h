#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/planning/obstacle/obstacle.h"

namespace dharma {
namespace planning {

class PredictionQuerier {
public:
    PredictionQuerier(const std::vector<const Obstacle*>& obstacles,
                      const std::shared_ptr<std::vector<common::PathPoint>>& ptr_reference_line);

    virtual ~PredictionQuerier() = default;

    std::vector<const Obstacle*> GetObstacles() const;

    double ProjectVelocityAlongReferenceLine(const std::string& obstacle_id,
                                             const double s,
                                             const double t) const;

private:
    std::unordered_map<std::string, const Obstacle*> id_obstacle_map_;

    std::vector<const Obstacle*> obstacles_;

    std::shared_ptr<std::vector<common::PathPoint>> ptr_reference_line_;
};

}  // namespace planning
}  // namespace dharma
