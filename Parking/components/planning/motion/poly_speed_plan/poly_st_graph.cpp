#include "planning/motion/poly_speed_plan/poly_st_graph.h"

#include <fstream>

#include "common/util/util.h"
#include "common/math/linear_interpolation.h"
#include "common/vehicle_param_handle/vehicle_param_handle.h"

namespace planning {

using State = std::array<double, 3>;
using Condition = std::pair<State, double>;
typedef std::vector<std::shared_ptr<Curve1d>> SpeedProfilesBundle;

namespace {
constexpr auto kInfCost = std::numeric_limits<double>::infinity();
}

PolyStGraph::PolyStGraph(const PolySpeedPlanConfig& config) : poly_config_(config){
    CHECK_GE(poly_config_.sample_config().num_sample_follow_per_timestamp(), 2);
}

bool PolyStGraph::FindStTunnel(const common::TrajectoryPoint &init_point,
                               const PathData &min_cost_path,
                               SpeedData *const speed_data){
    init_lon_state_ = {init_point.path_point().s(), init_point.v(), init_point.a()};

    feasible_region_ = std::make_unique<FeasibleRegion>(init_lon_state_,
                                                        poly_config_.max_acceleration(),
                                                        poly_config_.max_deceleration());
    min_cost_path_ = min_cost_path;

    SpeedProfilesBundle speed_profiles_bundle;

    if(min_cost_path.IsTruncated()){
        GenerateSpeedProfilesForTruncated(&speed_profiles_bundle);
    }
    else {
        GenerateSpeedProfilesForCruising(&speed_profiles_bundle);
    }

    if(speed_profiles_bundle.empty()){
        return false;
    }



    constexpr double delta_t = 0.1;
    uint32_t counter = 0;
    double min_cost = kInfCost;
    uint32_t min_index = std::numeric_limits<uint32_t>::infinity();
    for(auto profile : speed_profiles_bundle){

        double cost = CalculateLonCost(profile);
        if(cost < min_cost){
            min_cost = cost;
            min_index = counter;
        }

//        std::string fname;

//        if(min_cost_path_.IsTruncated()){
//            fname = "speed_points_truncated_" + std::to_string(counter) + ".log";
//        }
//        else {
//            fname = "speed_points_" + std::to_string(counter) + ".log";
//        }

//        std::fstream openfile(fname, std::ios::ate|std::ios::out);

//        for(double t = 0.0; t <= poly_config_.sample_config().time_length(); t+=delta_t){
//            double s = profile->Evaluate(0,t);
//            double v = profile->Evaluate(1,t);
//            double a = profile->Evaluate(2,t);
//            double da = profile->Evaluate(3,t);

//            openfile << s << " "
//                     << t << " "
//                     << v << " "
//                     << a << " "
//                     << da << "\n";
//        }
//        openfile.close();
        counter ++;
    }

    std::string min_speed_file = "min_cost_speed.log";
    std::fstream minopenfile(min_speed_file, std::ios::ate|std::ios::out);

    auto min_profile = speed_profiles_bundle.at(min_index);

    for(double t = 0.0; t < poly_config_.sample_config().time_length(); t+=delta_t){
        double s = min_profile->Evaluate(0,t);
        double v = min_profile->Evaluate(1,t);
        double a = min_profile->Evaluate(2,t);
        double da = min_profile->Evaluate(3,t);

        minopenfile << s << " "
                 << t << " "
                 << v << " "
                 << a << " "
                 << da << "\n";

        speed_data->AppendSpeedPoint(s, t, v, a, da);
    }

    minopenfile.close();


//    std::cout << "min_index: " << min_index << std::endl;

    return true;
}

void PolyStGraph::GenerateSpeedProfilesForTruncated(SpeedProfilesBundle *ptr_speed_profiles_bundle) const {
    auto end_conditions = SampleEndConditionsForTruncated(min_cost_path_.discretized_path().back().s());
    if(end_conditions.empty()){
        return;
    }
    GenerateSpeedProfileBundle<5>(init_lon_state_, end_conditions, ptr_speed_profiles_bundle);

    return;
}

std::vector<Condition> PolyStGraph::SampleEndConditionsForTruncated(const double& stop_s) const {

    uint32_t num_of_time_samples = static_cast<uint32_t>(
                poly_config_.sample_config().time_length() / poly_config_.sample_config().time_density()) + 1;

    std::vector<double> time_samples;

    common::util::uniform_slice(0.0,
                                poly_config_.sample_config().time_length(),
                                num_of_time_samples - 1,
                                &time_samples);
    time_samples[0] = 0.1;

    std::vector<Condition> end_s_conditions;

    for(const auto& time : time_samples){
        double s_upper = stop_s - common::VehicleParamHandle::GetParam().front_edge_to_center();
        double s_lower = s_upper - poly_config_.sample_config().lon_buffer();
        double s_gap =
                poly_config_.sample_config().lon_buffer() /
                static_cast<double>(poly_config_.sample_config().num_sample_follow_per_timestamp() - 1);

        for(int32_t i = 0; i < poly_config_.sample_config().num_sample_follow_per_timestamp(); ++i){
            double s = s_lower + s_gap * static_cast<double>(i);
            if (s > feasible_region_->SUpper(time) || s < feasible_region_->SLower(time)) {
                continue;
            }

            State end_state = {s, 0.0, 0.0};
            end_s_conditions.emplace_back(end_state, time);
        }
    }

    return end_s_conditions;
}


void PolyStGraph::GenerateSpeedProfilesForCruising(SpeedProfilesBundle *ptr_speed_profiles_bundle) const {
    auto end_conditions = SampleEndConditionsForCrusing();

    if(end_conditions.empty()){
        return;
    }

    GenerateSpeedProfileBundle<4>(init_lon_state_, end_conditions, ptr_speed_profiles_bundle);

    return;
}

std::vector<Condition> PolyStGraph::SampleEndConditionsForCrusing() const {

    uint32_t num_of_time_samples = static_cast<uint32_t>(
                poly_config_.sample_config().time_length() / poly_config_.sample_config().time_density()) + 1;

    std::vector<double> time_samples;

    common::util::uniform_slice(0.0,
                                poly_config_.sample_config().time_length(),
                                num_of_time_samples - 1,
                                &time_samples);
    time_samples[0] = 0.1;

    std::vector<Condition> end_s_conditions;

    const double top_speed = poly_config_.top_speed();

    for(const auto& time : time_samples){
        double v_upper = feasible_region_->VUpper(time);
        double v_lower = feasible_region_->VLower(time);

        if(v_upper >= top_speed && v_lower <= top_speed){
            State end_s = {0.0, top_speed, 0.0};
            end_s_conditions.emplace_back(end_s, time);
        }

    }

    return end_s_conditions;
}

double PolyStGraph::CalculateLonCost(const std::shared_ptr<Curve1d> &speed_profile) const {
    double cost_sum = 0.0;

    double time_length = poly_config_.sample_config().time_length();
    double time_gap = poly_config_.sample_config().time_density();
    double top_speed = poly_config_.top_speed();
    double max_accel = poly_config_.max_acceleration();
    double max_decel = poly_config_.max_deceleration();


    for(double t = 0.0; t < time_length; t+=time_gap){
        double jerk = speed_profile->Evaluate(3, t);
        double accel = speed_profile->Evaluate(2, t);
        double speed = speed_profile->Evaluate(1, t);

        if(speed < 0){
            return kInfCost;
        }

        if(speed > init_lon_state_[1] && speed > poly_config_.top_speed()){
            return kInfCost;
        }

        cost_sum += LonSpeedCost(speed, top_speed, time_gap);
        cost_sum += LonAccelCost(accel, max_accel, max_decel, time_gap);
        cost_sum += LonJerkCost(jerk, time_gap);
    }

    return cost_sum;
}

double PolyStGraph::LonSpeedCost(const double &speed, const double& top_speed, const double& time_gap) const {
    double cost = 0.0;
    if(speed < top_speed && !min_cost_path_.IsTruncated()){
        cost += poly_config_.keep_clear_low_speed_penalty() * time_gap * poly_config_.default_speed_cost();
    }

    double det_speed = (speed - top_speed)/top_speed;
    if(det_speed > 0){
        cost += poly_config_.exceed_speed_penalty() * poly_config_.default_speed_cost() * speed * speed * time_gap;
    }
    else if (det_speed < 0) {
        cost += poly_config_.low_speed_penalty() * poly_config_.default_speed_cost() * (-det_speed) * time_gap;
    }
    return cost;
}

double PolyStGraph::LonAccelCost(const double &accel, const double &max_acc,
                               const double &max_dec, const double &time_gap) const {
    double cost = 0.0;

    const double accel_sq = accel * accel;

    if (accel > 0.0) {
      cost = poly_config_.accel_penalty() * accel_sq;
    } else {
      cost = poly_config_.decel_penalty() * accel_sq;
    }

    cost += accel_sq * poly_config_.decel_penalty() * poly_config_.decel_penalty() /(1 + std::exp(1.0 * (accel - max_dec))) +
                accel_sq * poly_config_.accel_penalty() * poly_config_.accel_penalty() /(1 + std::exp(-1.0 * (accel - max_acc)));

    return cost * time_gap;
}

double PolyStGraph::LonJerkCost(const double &jerk, const double& time_gap) const{
    double cost = 0.0;
    double jerk_sq = jerk * jerk;
    if(jerk > 0){
        cost = poly_config_.positive_jerk_coeff() * jerk_sq * time_gap;
    }
    else {
        cost = poly_config_.negative_jerk_coeff() * jerk_sq * time_gap;
    }
    return cost;
}

}
