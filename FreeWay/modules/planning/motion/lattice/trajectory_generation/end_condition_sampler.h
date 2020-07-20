#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/speed/st_point.h"
#include "modules/planning/motion/lattice/behavior/feasible_region.h"
#include "modules/planning/motion/lattice/behavior/path_time_graph.h"
#include "modules/planning/motion/lattice/behavior/prediction_querier.h"

namespace dharma {
namespace planning {

struct SamplePoint {
    STPoint path_time_point;
    double ref_v;
};

// Input: planning objective, vehicle kinematic/dynamic constraints,
// Output: sampled ending 1 dimensional states with corresponding time duration.
class EndConditionSampler {
public:
    EndConditionSampler(
            const std::array<double, 3>& init_s, const std::array<double, 3>& init_d,
            std::shared_ptr<PathTimeGraph> ptr_path_time_graph,
            std::shared_ptr<PredictionQuerier> ptr_prediction_querier);

    virtual ~EndConditionSampler() = default;

    std::vector<std::pair<std::array<double, 3>, double>> SampleLatEndConditions()
    const;

    std::vector<std::pair<std::array<double, 3>, double>>
    SampleLonEndConditionsForCruising(const double ref_cruise_speed) const;

    std::vector<std::pair<std::array<double, 3>, double>>
    SampleLonEndConditionsForStopping(const double ref_stop_point) const;

    std::vector<std::pair<std::array<double, 3>, double>>
    SampleLonEndConditionsForPathTimePoints() const;

private:
    std::vector<SamplePoint> QueryPathTimeObstacleSamplePoints() const;

    void QueryFollowPathTimePoints(
            const common::VehicleConfig& vehicle_config,
            const std::string& obstacle_id,
            std::vector<SamplePoint>* sample_points) const;

    void QueryOvertakePathTimePoints(
            const common::VehicleConfig& vehicle_config,
            const std::string& obstacle_id,
            std::vector<SamplePoint>* sample_points) const;

private:
    std::array<double, 3> init_s_;
    std::array<double, 3> init_d_;
    FeasibleRegion feasible_region_;
    std::shared_ptr<PathTimeGraph> ptr_path_time_graph_;
    std::shared_ptr<PredictionQuerier> ptr_prediction_querier_;
};

}  // namespace planning
}  // namespace dharma
