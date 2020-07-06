#ifndef PLANNING_MOTION_POLY_SPEED_PLAN_POLY_ST_GRAPH_H_
#define PLANNING_MOTION_POLY_SPEED_PLAN_POLY_ST_GRAPH_H_

#include "common/proto/pnc_point.pb.h"
#include "common/mlog/mlog.h"

#include "planning/common/path/path_data.h"
#include "planning/common/speed/speed_data.h"
#include "planning/math/curve1d/curve1d.h"
#include "planning/math/curve1d/quartic_polynomial_curve1d.h"
#include "planning/math/curve1d/quintic_polynomial_curve1d.h"
#include "planning/reference_line/reference_line.h"
#include "planning/motion/poly_speed_plan/feasible_region.h"
#include "planning/motion/poly_speed_plan/speed_profile.h"
#include "planning/proto/poly_speed_plan_config.pb.h"

namespace planning {

class PolyStGraph
{
public:
    explicit PolyStGraph(const PolySpeedPlanConfig& config);

    ~PolyStGraph() = default;

public:

    bool FindStTunnel(const common::TrajectoryPoint& init_point,
                      const PathData& min_cost_path,
                      SpeedData *const speed_data);

private:
    // generate profile
    void GenerateSpeedProfilesForTruncated(std::vector<std::shared_ptr<Curve1d>>* ptr_speed_profiles_bundle) const;

    std::vector<std::pair<std::array<double, 3>, double>> SampleEndConditionsForTruncated(const double& truncated_s) const;

    void GenerateSpeedProfilesForCruising(std::vector<std::shared_ptr<Curve1d>>* ptr_speed_profiles_bundle) const;

    std::vector<std::pair<std::array<double, 3>, double>> SampleEndConditionsForCrusing() const;

    template <size_t N>
    void GenerateSpeedProfileBundle(const std::array<double, 3>& init_state,
                                    const std::vector<std::pair<std::array<double, 3>, double>>& end_conditions,
                                    std::vector<std::shared_ptr<Curve1d>>* ptr_trajectory_bundle) const;

    // calculate cost
    double CalculateLonCost(const std::shared_ptr<Curve1d>& speed_profile) const;


    double LonSpeedCost(const double& speed, const double& top_speed, const double& time_gap) const;
    double LonAccelCost(const double& acc, const double& max_acc, const double& max_dec, const double& time_gap) const;


    double LonJerkCost(const double& jerk, const double& time_gap) const;


private:
    std::array<double, 3> init_lon_state_; // s, s'(v), s''(a)

    std::unique_ptr<FeasibleRegion> feasible_region_;

    PathData min_cost_path_;

private:
    // config info
    PolySpeedPlanConfig poly_config_;
    double keep_clear_low_speed_penalty_ = 10.0;
    double default_speed_cost_ = 1000.0;
    double exceed_speed_penalty_ = 10.0;
    double low_speed_penalty_ = 10.0;
    double accel_penalty_ = 1.0;
    double decel_penalty_ = 1.0;
    double positive_jerk_coeff_ = 1.0;
    double negative_jerk_coeff_ = 1.0;
};

template<>
inline void PolyStGraph::GenerateSpeedProfileBundle<4>(const std::array<double, 3>& init_state,
                                                       const std::vector<std::pair<std::array<double, 3>, double>>& end_conditions,
                                                       std::vector<std::shared_ptr<Curve1d>>* ptr_speed_profile_bundle) const{
    CHECK_NOTNULL(ptr_speed_profile_bundle);
    CHECK(!end_conditions.empty());

    ptr_speed_profile_bundle->reserve(ptr_speed_profile_bundle->size() + end_conditions.size());

    for(const auto& end_condition : end_conditions){
        auto ptr_profile = std::make_shared<SpeedProfile>(
                    std::shared_ptr<Curve1d>(
                        new QuarticPolynomialCurve1d(
                            init_state, {end_condition.first[1], end_condition.first[2]}, end_condition.second)
                        )
                    );
        ptr_profile->set_target_velocity(end_condition.first[1]);
        ptr_profile->set_target_time(end_condition.second);
        ptr_speed_profile_bundle->push_back(ptr_profile);
    }
}

template<>
inline void PolyStGraph::GenerateSpeedProfileBundle<5>(const std::array<double, 3>& init_state,
                                                       const std::vector<std::pair<std::array<double, 3>, double>>& end_conditions,
                                                       std::vector<std::shared_ptr<Curve1d>>* ptr_speed_profile_bundle) const{
    CHECK_NOTNULL(ptr_speed_profile_bundle);
    CHECK(!end_conditions.empty());

    ptr_speed_profile_bundle->reserve(ptr_speed_profile_bundle->size() + end_conditions.size());

    for(const auto& end_condition : end_conditions){
        auto ptr_profile = std::make_shared<SpeedProfile>(
                    std::shared_ptr<Curve1d>(
                        new QuinticPolynomialCurve1d(
                            init_state, end_condition.first, end_condition.second)
                        )
                    );
        ptr_profile->set_target_position(end_condition.first[0]);
        ptr_profile->set_target_velocity(end_condition.first[1]);
        ptr_profile->set_target_time(end_condition.second);
        ptr_speed_profile_bundle->push_back(ptr_profile);
    }
}

}
#endif
