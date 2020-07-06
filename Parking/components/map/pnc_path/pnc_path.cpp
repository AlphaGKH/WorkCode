#include "map/pnc_path/pnc_path.h"

#include "common/util/file.h"
#include "common/math/linear_interpolation.h"

namespace map {

PncPath::PncPath(RoadMap* const road_map): road_map_(road_map){}

bool PncPath::Init(){
    CHECK_NOTNULL(road_map_);
    if(!road_map_->Init()){
        return false;
    }

    map_path_ = std::make_unique<MapPath>(road_map_->map_path_points());

    if(!common::util::GetProtoFromFile(FLAGS_pnc_path_config_filename, &config_)){
        AERROR << "PncPath unable to parse pnc_path_config_file" << FLAGS_pnc_path_config_filename;
        return false;
    }

    if(!InitLaneSegmentKDTree()){
        return false;
    }

    return true;

}

bool PncPath::InitLaneSegmentKDTree(){
    if(map_path_->segments().empty()){
        AERROR << "Segments is Empty, can't build a KDTree!";
        return false;
    }

    common::math::AABoxKDTreeParams params;
    params.max_leaf_dimension = 5.0;  // meters.
    params.max_leaf_size = 16;

    lane_segment_boxes_.clear();
    for(size_t index = 0; index < map_path_->num_segments(); ++index){
        const auto& segment = map_path_->segments()[index];
        lane_segment_boxes_.emplace_back(
                    common::math::AABox2d(segment.start(), segment.end()),
                    &segment, index);
    }

    lane_segment_kdtree_ = std::make_unique<LaneSegmentKDTree>(lane_segment_boxes_, params);

    return true;

}

template <class KDTree>
bool PncPath::SearchObjects(const common::math::Vec2d &center, const double radius,
                            const KDTree &kdtree, std::vector<int> *const result_indices){
    if (result_indices == nullptr)
    {
        return false;
    }
    auto objects = kdtree.GetObjects(center, radius);
    std::unordered_set<double> results;
    results.reserve(objects.size());
    for (const auto *object_ptr : objects)
    {
        results.insert(object_ptr->index());
    }

    result_indices->reserve(results.size());
    result_indices->assign(results.begin(), results.end());
    return true;
}

bool PncPath::GetNearbyPoints(const common::VehicleState &vehicle_state,
                              std::vector<map::MapPathPoint> *points){
    int adc_segment_index = 0;
    double adc_s = 0;
    double adc_l = 0;
    if(!GetNearestPointWithTheta({vehicle_state.x(), vehicle_state.y()}, vehicle_state.theta(),
                                 config_.search_radius(), config_.search_theta_diff(),
                                 &adc_segment_index, &adc_s, &adc_l)){
        AERROR << "Failed to get nearest map_path_point from :"
               << vehicle_state.DebugString()
               << "search_radius: " << config_.search_radius()
               << "max_theta_diff: " << config_.search_theta_diff();
        return false;
    }

    const auto& start_s = adc_s - config_.look_backward_distance();
    const auto& end_s = adc_s  + config_.look_forward_distance();

    if(end_s - map_path_->accumulated_s().back() > config_.look_forward_distance()){
        stop_for_destination_ = true;
    }

    if(!ExtendSegments(adc_segment_index, start_s, end_s, points)){
        return false;
    }

    return true;


}

bool PncPath::GetNearestPointWithTheta(const common::math::Vec2d &point, const double &central_theta,
                                       const double &max_distance, const double &max_theta_diff,
                                       int *nearest_index, double* s, double* l) const{
    CHECK_NOTNULL(nearest_index);
    if(lane_segment_kdtree_ == nullptr){
        AERROR << "LaneSegmentKDTree is nullptr!";
        return false;
    }

    std::vector<int> distance_segments_indexes;
    if(!SearchObjects(point, max_distance, *lane_segment_kdtree_, &distance_segments_indexes)){
        AERROR << "Don't get lane_segment from point: " << point.DebugString()
               << "in range of: " << max_distance << "m";
        return false;
    }

    double min_dis = max_distance;

    for(const auto& i : distance_segments_indexes){
        const auto& segment = map_path_->segments()[i];
        double theta_diff = common::math::NormalizeAngle(fabs(central_theta - segment.theta()));
        if(theta_diff <= max_theta_diff){
            double dis = segment.DistanceTo(point);
            if(dis < min_dis){
                min_dis = dis;
                *nearest_index = i;
            }
        }
    }

    if(nearest_index == nullptr || *nearest_index < 0 || *nearest_index > map_path_->num_segments() - 1){
        return false;
    }

    common::math::Vec2d projection_point;
    double delta_s = 0.0;

    const auto& nearest_seg = map_path_->segments()[*nearest_index];
    const double& nearest_distance = nearest_seg.DistanceTo(point, &projection_point);

    delta_s = projection_point.DistanceTo(nearest_seg.start());

    *s = map_path_->accumulated_s()[*nearest_index] + delta_s;
    *l = nearest_seg.unit_direction().CrossProd(point - nearest_seg.start());

    if(*l > config_.search_radius()){
        return false;
    }

    return true;
}

bool PncPath::ExtendSegments(const int& central_index, const double &start_s,
                             const double &end_s, std::vector<MapPathPoint> *points) const{
    CHECK_NOTNULL(points);
    points->clear();
    static constexpr double kLengthEpsilon = 1e-3;

    if (start_s + kLengthEpsilon >= end_s) {
        AERROR << "start_s(" << start_s << " >= end_s(" << end_s << ")";
        return false;
    }

    if(start_s < 0){
        const auto& first_segment = map_path_->segments().begin();

        const auto& first_point = common::math::Vec2d(map_path_->points().begin()->x(), map_path_->points().begin()->y());

        const common::math::Vec2d& backward_point = first_point + first_segment->unit_direction() * start_s;

        points->emplace_back(map_path_->points().front());
        points->back().set_x(backward_point.x());
        points->back().set_y(backward_point.y());
    }

    //
    const double head_s = std::max(0.0, start_s);

    int prev_index = central_index;

    while (map_path_->accumulated_s()[prev_index] > head_s) {
        if(prev_index <= 0){
            break;
        }
        else {
            --prev_index;
        }
    }

    const double tail_s = std::min(end_s, map_path_->accumulated_s().back());

    int next_index = central_index ;

    while (tail_s > map_path_->accumulated_s()[next_index]) {
        if(next_index + 1 >= map_path_->num_points()){
            break;
        }
        else {
            next_index ++;
        }
    }

    if(fabs(head_s - map_path_->accumulated_s()[prev_index]) < kLengthEpsilon){
        points->emplace_back(map_path_->points()[prev_index]);
    }
    else {
        points->emplace_back(LinearInterpolation(map_path_->accumulated_s()[prev_index],
                                                 map_path_->points()[prev_index],
                                                 map_path_->accumulated_s()[prev_index + 1],
                             map_path_->points()[prev_index + 1],
                head_s));
    }

    for(auto i = prev_index + 1; i < next_index; ++i){
        points->emplace_back(map_path_->points()[i]);
    }

    if(fabs(tail_s - map_path_->accumulated_s()[next_index]) < kLengthEpsilon){
        points->emplace_back(map_path_->points()[next_index]);
    }
    else {
        points->emplace_back(LinearInterpolation(map_path_->accumulated_s()[next_index - 1],
                             map_path_->points()[next_index - 1],
                map_path_->accumulated_s()[next_index],
                map_path_->points()[next_index],
                tail_s));
    }

    //
    if(tail_s < end_s){
        const auto& last_segment = map_path_->segments().back();
        const auto& last_point = common::math::Vec2d(map_path_->points().back().x(), map_path_->points().back().y());

        const common::math::Vec2d& forward_point = last_point + last_segment.unit_direction() * (end_s - tail_s);

        points->emplace_back(map_path_->points().back());
        points->back().set_x(forward_point.x());
        points->back().set_y(forward_point.y());
    }

    if(points->empty()){
        return false;
    }

    return true;
}

MapPathPoint PncPath::LinearInterpolation(const double &prev_s, const MapPathPoint &prev_point,
                                          const double &next_s, const MapPathPoint &next_point,
                                          const double &target_s) const{
    CHECK_LE(prev_s,next_s);

    MapPathPoint point(prev_point);

    double weight = (target_s - prev_s)/(next_s - prev_s);

    double x = (1 - weight) * prev_point.x() + weight * next_point.x();
    double y = (1 - weight) * prev_point.y() + weight * next_point.y();
    double theta = common::math::slerp(prev_point.theta(), prev_s, next_point.theta(), next_s, target_s);

    point.set_x(x);
    point.set_y(y);
    point.set_theta(theta);

    return point;
}

}
