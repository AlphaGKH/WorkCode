#include "planning/reference_line/reference_line_provider.h"

#include "common/time/time.h"
#include "common/util/file.h"
#include "common/util/util.h"
#include "common/math/math_utils.h"
#include "common/vehicle_param_handle/vehicle_param_handle.h"

#include "planning/reference_line/qp_spline_reference_line_smoother.h"

namespace planning {

ReferenceLineProvider::ReferenceLineProvider(map::RoadMap* const road_map, const ReferenceLineProviderConfig& config){
    CHECK_NOTNULL(road_map);
    if(!road_map->Init()){
        AERROR << "ReferenceLineProvider Init Failed!";
        is_inited_ = false;
        is_stop_ = true;
    }
    else {
        is_inited_ = true;
        is_stop_ = false;
    }

    pnc_path_ = std::make_unique<map::PncPath>(road_map);

    if(!pnc_path_->Init()){
        is_inited_ = false;
        is_stop_ = true;
    }
    else {
        is_inited_ = true;
        is_stop_ = false;
    }

    provider_config_ = config;
    smoother_ = std::make_unique<QpSplineReferenceLineSmoother>(provider_config_.smooth_config());
}

ReferenceLineProvider::~ReferenceLineProvider(){
    if(reference_line_provider_thread_ptr_ && reference_line_provider_thread_ptr_->joinable()){
        reference_line_provider_thread_ptr_->join();
    }
}

void ReferenceLineProvider::UpdateVehicleState(const common::VehicleState &adc_state){
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);

    constexpr double kEpsion = 1e-2;
    if(common::util::DistanceXY(adc_state, vehicle_state_) < kEpsion){
        vehicle_state_update_ = false;
    }
    else {
        vehicle_state_ = adc_state;
        vehicle_state_update_ = true;
    }
}

bool ReferenceLineProvider::Start(){
    if (!is_inited_) {
        AERROR << "ReferenceLineProvider has NOT been initiated.";
        return false;
    }

    reference_line_provider_thread_ptr_ = std::make_unique<std::thread>
            (&ReferenceLineProvider::ReferenceLineProviderThread, this);

    return true;

}

void ReferenceLineProvider::ReferenceLineProviderThread(){
    constexpr int32_t kSleepTime = 50; // ms
    while (!is_stop_) {
        std::this_thread::yield();

        if(last_calculation_time_ > kSleepTime){
            AWARN << "It took Too Long Time for generating a reference line in last cycle!";
        }
        else {
            std::this_thread::sleep_for(
                        std::chrono::duration<double, std::milli>(kSleepTime - last_calculation_time_ * 1000));
        }
        double start_time = common::time::Clock::NowInSeconds();

        // generate new reference_line

        if(!vehicle_state_update_){
            UpdateReferenceLine(last_reference_line_);
            continue;
        }

        ReferenceLine ref_line;

        if(!CreatReferenceLine(&ref_line)){
            AERROR << "Failed to get reference_line";
            continue;
        }

        UpdateReferenceLine(ref_line);

        // generate reference_line end

        double end_time = common::time::Clock::NowInSeconds();

        last_calculation_time_ = end_time - start_time;
    }
}

void ReferenceLineProvider::Stop(){
    is_stop_ = true;
    if(reference_line_provider_thread_ptr_&&reference_line_provider_thread_ptr_->joinable()){
        reference_line_provider_thread_ptr_->join();
    }
}

bool ReferenceLineProvider::GetReferenceLine(ReferenceLine *ref_line){
    CHECK_NOTNULL(ref_line);

    if(update_counter_ > 0){
        std::lock_guard<std::mutex> lock(reference_line_mutex_);
        *ref_line = reference_line_;
        last_reference_line_ = reference_line_;
        --update_counter_;
    }
    else {
        AERROR << "Planning module has not get ReferenceLine for " << 10 << " cycles";
        return false;
    }
    return true;
}

bool ReferenceLineProvider::CreatNearbyPoints(const common::VehicleState &vehicle_state,
                                              std::vector<map::MapPathPoint>* points){
    if(!pnc_path_->GetNearbyPoints(vehicle_state, points)){
        return false;
    }

    return true;

}

bool ReferenceLineProvider::CreatReferenceLine(ReferenceLine *reference_line){
    CHECK_NOTNULL(reference_line);
    std::vector<map::MapPathPoint> map_points;

    common::VehicleState vehicle_state;
    {
        std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
        vehicle_state = vehicle_state_;
    }

    if(!CreatNearbyPoints(vehicle_state,&map_points)){
        AERROR << "Failed to create nearby map_path_points from vehicle_state; " << vehicle_state.DebugString();
        return false;
    }

    if(!provider_config_.enable_reference_line_stitching()){
        if(!SmoothReferenceLine(ReferenceLine(map::MapPath(map_points)), reference_line)){
            AERROR << "Failed to create reference line from map_path_points";
        }
        else {
            common::SLPoint sl;
            if(!reference_line->XYToSL(common::math::Vec2d(vehicle_state.x(), vehicle_state.y()), &sl)){
                AWARN << "Failed to project point: {" << vehicle_state.x() << ","
                      << vehicle_state.y() << "} to stitched reference line";
            }

            Shrink(sl, reference_line);
        }

        return true;
    }
    else {
        // stitching reference_line

    }

    return true;

}

bool ReferenceLineProvider::SmoothReferenceLine(const ReferenceLine &raw_reference_line,
                                                ReferenceLine *reference_line){
    if(!provider_config_.enable_smooth_reference_line()){
        constexpr double kSampleDistance = 1.0;
        std::vector<ReferencePoint> ref_points;

        uint32_t points_number = static_cast<uint32_t>(raw_reference_line.Length() / kSampleDistance);

        ref_points.reserve(points_number);

        for(uint32_t i = 0; i < points_number; ++i){
            ref_points.emplace_back(raw_reference_line.GetReferencePoint(i * kSampleDistance));
        }

        *reference_line = ReferenceLine(ref_points);
//        *reference_line = raw_reference_line;
        return true;
    }

    // smoother reference_line here
    std::vector<AnchorPoint> anchor_points;
    GetAnchorPoints(raw_reference_line, &anchor_points);
    smoother_->SetAnchorPoints(anchor_points);

    if (!smoother_->Smooth(raw_reference_line, reference_line)) {
        AERROR << "Failed to smooth reference line with anchor points";
        return false;
    }

    if (!IsReferenceLineSmoothValid(raw_reference_line, *reference_line)) {
        AERROR << "The smoothed reference line error is too large";
        return false;
    }

    return true;
}

bool ReferenceLineProvider::Shrink(const common::SLPoint &sl, ReferenceLine *reference_line){
    static constexpr double kMaxThetaDiff = M_PI * 5.0 / 6.0;
    // shrink reference line
    double new_backward_distance = sl.s();
    double new_forward_distance = reference_line->Length() - sl.s();
    bool need_shrink = false;

    if (sl.s() > 40/*FLAGS_default_look_backward*/ * 1.5) {
        ADEBUG << "reference line back side is " << sl.s()
               << ", shrink reference line: origin length: "
               << reference_line->Length();
        new_backward_distance = 40/*FLAGS_default_look_backward*/;
        need_shrink = true;
    }
    // check theta
    const auto index = reference_line->GetNearestReferenceIndex(sl.s());
    const auto &ref_points = reference_line->reference_points();
    const double cur_theta = ref_points[index].theta();

    auto last_index = index;
    while (last_index < ref_points.size() &&
           common::math::AngleDiff(cur_theta, ref_points[last_index].theta()) < kMaxThetaDiff) {
        ++last_index;
    }
    --last_index;
    if (last_index != ref_points.size() - 1) {
        need_shrink = true;
        common::SLPoint forward_sl;
        reference_line->XYToSL(ref_points[last_index], &forward_sl);
        new_forward_distance = forward_sl.s() - sl.s();
    }

    if(need_shrink){
        if (!reference_line->Segment(sl.s(), new_backward_distance,
                                     new_forward_distance)) {
            AWARN << "Failed to shrink reference line";
        }
    }

    return true;
}

void ReferenceLineProvider::UpdateReferenceLine(const ReferenceLine &reference_line){
    std::lock_guard<std::mutex> lock(reference_line_mutex_);
    reference_line_ = reference_line;
    update_counter_ = provider_config_.update_counter();

}

void ReferenceLineProvider::GetAnchorPoints(const ReferenceLine &reference_line,
                                            std::vector<AnchorPoint> *anchor_points) const{
    CHECK_NOTNULL(anchor_points);
    const double interval = provider_config_.smooth_config().max_constraint_interval();
    int num_of_anchors =
            std::max(2, static_cast<int>(reference_line.Length() / interval + 0.5));
    std::vector<double> anchor_s;
    common::util::uniform_slice(0.0, reference_line.Length(), num_of_anchors - 1,
                                &anchor_s);
    for (const double s : anchor_s) {
        anchor_points->emplace_back(GetAnchorPoint(reference_line, s));
    }
    anchor_points->front().longitudinal_bound = 1e-6;
    anchor_points->front().lateral_bound = 1e-6;
    anchor_points->front().enforced = true;
    anchor_points->back().longitudinal_bound = 1e-6;
    anchor_points->back().lateral_bound = 1e-6;
    anchor_points->back().enforced = true;
}

AnchorPoint ReferenceLineProvider::GetAnchorPoint(const ReferenceLine &reference_line, double s) const{
    AnchorPoint anchor;
    anchor.longitudinal_bound = provider_config_.smooth_config().longitudinal_boundary_bound();

    auto ref_point = reference_line.GetReferencePoint(s);

    const double adc_width = common::VehicleParamHandle::GetParam().width();
    const double adc_half_width = adc_width / 2.0;

    double left_width = ref_point.lane_left_width();
    double right_width = ref_point.lane_right_width();

    double total_width = left_width + right_width;

    double shifted_left_width = total_width / 2.0;

    if(total_width > adc_width * provider_config_.smooth_config().wide_lane_threshold_factor()){
        shifted_left_width = adc_half_width +
                adc_width * provider_config_.smooth_config().wide_lane_shift_remain_factor();
    }

    auto shifted_right_width = total_width - shifted_left_width;
    double effective_width = std::min(shifted_left_width, shifted_right_width) -
            adc_half_width - 0.5;

    anchor.path_point = ref_point.ToPathPoint(s);
    anchor.lateral_bound =
            std::max(provider_config_.smooth_config().lateral_boundary_bound(), effective_width);
    return anchor;
}

bool ReferenceLineProvider::IsReferenceLineSmoothValid(const ReferenceLine &raw,
                                                       const ReferenceLine &smoothed) const{
    static constexpr double kReferenceLineDiffCheckStep = 10.0;
    for (double s = 0.0; s < smoothed.Length();
         s += kReferenceLineDiffCheckStep) {
        auto xy_new = smoothed.GetReferencePoint(s);
        common::SLPoint sl_new;
        if (!raw.XYToSL(xy_new, &sl_new)) {
            AERROR << "Fail to change xy point on smoothed reference line to sl "
                      "point respect to raw reference line.";
            return false;
        }

        const double diff = std::fabs(sl_new.l());
        if (diff > provider_config_.smoothed_reference_line_max_diff()) {
            AERROR << "Fail to provide reference line because too large diff "
                      "between smoothed and raw reference lines. diff: "
                   << diff;
            return false;
        }
    }
    return true;

}

}
