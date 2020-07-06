#include "map/map_path/map_path.h"

#include <unordered_set>
#include <algorithm>

#include "common/mlog/mlog.h"
#include "common/math/math_utils.h"
#include "common/math/linear_interpolation.h"

namespace map {

MapPath::MapPath(const std::vector<MapPathPoint>& map_path_points)
    : points_(map_path_points){
    Init();
}

MapPath::MapPath(std::vector<MapPathPoint>&& map_path_points)
    : points_(std::move(map_path_points)){
    Init();
}

void MapPath::Init(){
    InitPoints();
}

void MapPath::InitPoints(){
    num_points_ = static_cast<size_t>(points_.size());
    CHECK_GE(num_points_, 2);

    accumulated_s_.clear();
    accumulated_s_.reserve(num_points_);
    segments_.clear();
    segments_.reserve(num_points_);
    unit_directions_.clear();
    unit_directions_.reserve(num_points_);
    lane_left_widths_.clear();
    lane_left_widths_.reserve(num_points_);
    lane_right_widths_.clear();
    lane_right_widths_.reserve(num_points_);

    double s = 0.0;

    for(size_t i = 0; i < num_points_; ++i){
        accumulated_s_.emplace_back(s);

        lane_left_widths_.emplace_back(points_[i].lane_left_width());
        lane_right_widths_.emplace_back(points_[i].lane_right_width());

        common::math::Vec2d vec;
        if(i + 1 >= num_points_){
            vec = points_[i] - points_[i - 1];
        }
        else {
            vec = points_[i + 1] - points_[i];
            s += vec.Length();
            segments_.emplace_back(points_[i], points_[i + 1]);
        }
        vec.Normalize();
        unit_directions_.emplace_back(vec);
    }

    length_ = s;
    num_segments_ = num_points_ - 1;

    CHECK_EQ(accumulated_s_.size(), num_points_);
    CHECK_EQ(unit_directions_.size(), num_points_);
    CHECK_EQ(segments_.size(), num_segments_);
}

bool MapPath::GetProjection(const common::math::Vec2d &point,double *accumulate_s,
                            double *lateral, double *min_distance) const{
    if (segments_.empty()) {
        return false;
    }
    if (accumulate_s == nullptr || lateral == nullptr ||
            min_distance == nullptr) {
        return false;
    }

    CHECK_GE(num_points_, 2);
    *min_distance = std::numeric_limits<double>::infinity();
    int min_index = 0;
    for (int i = 0; i < num_segments_; ++i) {
        const double distance = segments_.at(i).DistanceSquareTo(point);
        if (distance < *min_distance) {
            min_index = i;
            *min_distance = distance;
        }
    }

    *min_distance = std::sqrt(*min_distance);
    const auto& nearest_seg = segments_[min_index];
    const auto prod = nearest_seg.ProductOntoUnit(point);
    const auto proj = nearest_seg.ProjectOntoUnit(point);

    if (min_index == 0) {
        *accumulate_s = std::min(proj, nearest_seg.length());
        if (proj < 0) {
            *lateral = prod;
        } else {
            *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
        }
    } else if (min_index == num_segments_ - 1) {
        *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
        if (proj > 0) {
            *lateral = prod;
        } else {
            *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
        }
    } else {
        *accumulate_s = accumulated_s_[min_index] +
                std::max(0.0, std::min(proj, nearest_seg.length()));
        *lateral = (prod > 0.0 ? 1 : -1) * *min_distance;
    }
    return true;

}

bool MapPath::GetProjection(const common::math::Vec2d &point, double *accumulate_s, double *lateral) const{
    double distance = 0.0;
    return GetProjection(point, accumulate_s, lateral, &distance);
}

InterpolatedIndex MapPath::GetIndexFromS(double s) const{

    if (s <= 0.0) {
        return {0, 0.0};
    }
    CHECK_GT(num_points_, 0);
    if (s >= length_ && num_points_ > 0) {
        return {num_points_ - 1, 0.0};
    }

    size_t low_index = std::lower_bound(accumulated_s_.begin(), accumulated_s_.end(), s) - accumulated_s_.begin();

    if(low_index == 0){
        return {0, 0.0};
    }
    else {
        low_index -= 1;
    }

    double low_s = accumulated_s_[low_index];

    return {low_index, s - low_s};
}

MapPathPoint MapPath::GetSmoothPoint(const InterpolatedIndex &index) const{
    CHECK_GE(index.index, 0);
    CHECK_LT(index.index, num_points_);

    const auto& pre_point = points_[index.index];

    if(index.index >= num_segments_){
        return pre_point;
    }

    if(std::fabs(index.offset) > 1e-3){
        const MapPathPoint& next_point = points_[index.index +1];

        const double& pre_s = accumulated_s_[index.index];
        const double& next_s = accumulated_s_[index.index + 1];
        double s = pre_s + index.offset;

        double lane_left_width = common::math::lerp(pre_point.lane_left_width(), pre_s,
                                                    next_point.lane_left_width(), next_s, s);

        double lane_right_width = common::math::lerp(pre_point.lane_right_width(), pre_s,
                                                     next_point.lane_right_width(), next_s, s);

        const common::math::Vec2d delta = unit_directions_[index.index] * index.offset;
        auto point =  MapPathPoint(pre_point.x() + delta.x(), pre_point.y() + delta.y(), pre_point.theta(),
                            lane_left_width, lane_right_width);
        return point;
    }
    else {
        return pre_point;
    }

}

void MapPath::Clear(){
    points_.clear();
    num_points_ = 0;
    accumulated_s_.clear();
    segments_.clear();
    num_segments_ = 0;
    unit_directions_.clear();
    length_ = 0;
    lane_left_widths_.clear();
    lane_right_widths_.clear();
}

}
