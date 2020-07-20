#include "modules/planning/reference_line/reference_line.h"

#include "modules/common/math/linear_interpolation.h"
#include "modules/common/proto/pnc_point.pb.h"

#include "modules/planning/common/planning_gflags.h"

namespace dharma {

namespace planning {

ReferenceLine::ReferenceLine(const std::vector<ReferencePoint>& reference_points)
    : reference_points_(reference_points),
      map_path_(std::move(std::vector<hdmap::MapPathPoint>(
                              reference_points.begin(), reference_points.end()))) {
    CHECK_EQ(map_path_.num_points(), reference_points_.size());
}

ReferenceLine::ReferenceLine(const hdmap::Path& hdmap_path)
    : map_path_(hdmap_path) {
    for (const auto& point : hdmap_path.path_points()) {
        DCHECK(!point.lane_waypoints().empty());
        const auto& lane_waypoint = point.lane_waypoints()[0];
        reference_points_.emplace_back(
                    hdmap::MapPathPoint(point, point.heading(), lane_waypoint), 0.0, 0.0);
    }
    CHECK_EQ(map_path_.num_points(), reference_points_.size());
}


bool ReferenceLine::Segment(const common::math::Vec2d& point,
                            const double look_backward,
                            const double look_forward) {
    common::SLPoint sl;
    if (!XYToSL(point, &sl)) {
        AERROR << "Failed to project point: " << point.DebugString();
        return false;
    }
    return Segment(sl.s(), look_backward, look_forward);
}

bool ReferenceLine::Segment(const double s, const double look_backward,
                            const double look_forward) {
    const auto& accumulated_s = map_path_.accumulated_s();

    // inclusive
    auto start_index =
            std::distance(accumulated_s.begin(),
                          std::lower_bound(accumulated_s.begin(), accumulated_s.end(),
                                           s - look_backward));

    // exclusive
    auto end_index =
            std::distance(accumulated_s.begin(),
                          std::upper_bound(accumulated_s.begin(), accumulated_s.end(),
                                           s + look_forward));

    if (end_index - start_index < 2) {
        AERROR << "Too few reference points after shrinking.";
        return false;
    }

    reference_points_ =
            std::vector<ReferencePoint>(reference_points_.begin() + start_index,
                                        reference_points_.begin() + end_index);

    map_path_ = hdmap::Path(std::vector<hdmap::MapPathPoint>(
                            reference_points_.begin(), reference_points_.end()));
    return true;
}



bool ReferenceLine::GetLaneWidth(const double s, double *const lane_left_width,
                                 double *const lane_right_width) const{
    if (map_path_.path_points().empty()) {
        return false;
    }

    if (!map_path_.GetLaneWidth(s, lane_left_width, lane_right_width)) {
        return false;
    }

    return true;
}

double ReferenceLine::GetSpeedLimitFromS(const double s) const{
    for (const auto& speed_limit : speed_limit_) {
        if (s >= speed_limit.start_s && s <= speed_limit.end_s) {
            return speed_limit.speed_limit;
        }
    }

    const auto& map_path_point = GetReferencePoint(s);
    double speed_limit = FLAGS_planning_upper_speed_limit;
    for (const auto& lane_waypoint : map_path_point.lane_waypoints()) {
        if (lane_waypoint.lane == nullptr) {
            AWARN << "lane_waypoint.lane is nullptr.";
            continue;
        }
        speed_limit =
                std::fmin(lane_waypoint.lane->lane().speed_limit(), speed_limit);
    }
    return speed_limit;
}

ReferencePoint ReferenceLine::GetReferencePoint(const double s) const {
    const auto& accumulated_s = map_path_.accumulated_s();
    if (s < accumulated_s.front() - 1e-2) {
        AWARN << "The requested s: " << s << " < 0.";
        return reference_points_.front();
    }
    if (s > accumulated_s.back() + 1e-2) {
        AWARN << "The requested s: " << s
              << " > reference line length: " << accumulated_s.back();
        return reference_points_.back();
    }

    auto interpolate_index = map_path_.GetIndexFromS(s);

    size_t index = interpolate_index.id;
    size_t next_index = index + 1;
    if (next_index >= reference_points_.size()) {
        next_index = reference_points_.size() - 1;
    }

    const auto& p0 = reference_points_[index];
    const auto& p1 = reference_points_[next_index];

    const double s0 = accumulated_s[index];
    const double s1 = accumulated_s[next_index];
    return InterpolateWithMatchedIndex(p0, s0, p1, s1, interpolate_index);
}

ReferencePoint ReferenceLine::InterpolateWithMatchedIndex(
        const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
        const double s1, const hdmap::InterpolatedIndex& index) const {
    if (std::fabs(s0 - s1) < common::math::kMathEpsilon) {
        return p0;
    }
    double s = s0 + index.offset;
    DCHECK_LE(s0 - 1.0e-6, s) << "s: " << s << " is less than s0 : " << s0;
    DCHECK_LE(s, s1 + 1.0e-6) << "s: " << s << " is larger than s1: " << s1;

    auto map_path_point = map_path_.GetSmoothPoint(index);
    const double kappa = common::math::lerp(p0.kappa(), s0, p1.kappa(), s1, s);
    const double dkappa = common::math::lerp(p0.dkappa(), s0, p1.dkappa(), s1, s);

    return ReferencePoint(map_path_point, kappa, dkappa);
}

bool ReferenceLine::GetSLBoundary(const common::math::Box2d &box, SLBoundary *const sl_boundary) const {
    double start_s(std::numeric_limits<double>::max());
    double end_s(std::numeric_limits<double>::lowest());
    double start_l(std::numeric_limits<double>::max());
    double end_l(std::numeric_limits<double>::lowest());
    std::vector<common::math::Vec2d> corners;
    box.GetAllCorners(&corners);

    // The order must be counter-clockwise
    std::vector<common::SLPoint> sl_corners;
    for (const auto& point : corners) {
        common::SLPoint sl_point;
        if (!XYToSL(point, &sl_point)) {
            AERROR << "Failed to get projection for point: " << point.DebugString()
                   << " on reference line.";
            return false;
        }
        sl_corners.push_back(std::move(sl_point));
    }

    for (size_t i = 0; i < corners.size(); ++i) {
        auto index0 = i;
        auto index1 = (i + 1) % corners.size();
        const auto& p0 = corners[index0];
        const auto& p1 = corners[index1];

        const auto p_mid = (p0 + p1) * 0.5;
        common::SLPoint sl_point_mid;
        if (!XYToSL(p_mid, &sl_point_mid)) {
            AERROR << "Failed to get projection for point: " << p_mid.DebugString()
                   << " on reference line.";
            return false;
        }

        common::math::Vec2d v0(sl_corners[index1].s() - sl_corners[index0].s(),
                               sl_corners[index1].l() - sl_corners[index0].l());

        common::math::Vec2d v1(sl_point_mid.s() - sl_corners[index0].s(),
                               sl_point_mid.l() - sl_corners[index0].l());

        *sl_boundary->add_boundary_point() = sl_corners[index0];

        // sl_point is outside of polygon; add to the vertex list
        if (v0.CrossProd(v1) < 0.0) {
            *sl_boundary->add_boundary_point() = sl_point_mid;
        }
    }

    for (const auto& sl_point : sl_boundary->boundary_point()) {
        start_s = std::fmin(start_s, sl_point.s());
        end_s = std::fmax(end_s, sl_point.s());
        start_l = std::fmin(start_l, sl_point.l());
        end_l = std::fmax(end_l, sl_point.l());
    }

    sl_boundary->set_start_s(start_s);
    sl_boundary->set_end_s(end_s);
    sl_boundary->set_start_l(start_l);
    sl_boundary->set_end_l(end_l);
    return true;
}

bool ReferenceLine::XYToSL(const common::math::Vec2d &xy_point, common::SLPoint *const sl_point) const {
    double s = 0.0;
    double l = 0.0;
    if (!map_path_.GetProjection(xy_point, &s, &l)) {
        AERROR << "Cannot get nearest point from path.";
        return false;
    }
    sl_point->set_s(s);
    sl_point->set_l(l);
    return true;
}



}

}
