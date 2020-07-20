#include "modules/map/pnc_map/route_segments.h"

namespace dharma {

namespace hdmap {

namespace {

// Minimum error in lane segmentation.
constexpr double kSegmentationEpsilon = 0.2;
} // namespace

bool RouteSegments::IsOnSegment() const { return is_on_segment_; }

const std::string &RouteSegments::Id() const { return id_; }

bool RouteSegments::WithinLaneSegment(const LaneSegment &lane_segment,
                                      const LaneWaypoint &waypoint) {
  return waypoint.lane &&
         lane_segment.lane->id().id() == waypoint.lane->id().id() &&
         lane_segment.start_s - kSegmentationEpsilon <= waypoint.s &&
         lane_segment.end_s + kSegmentationEpsilon >= waypoint.s;
}

bool RouteSegments::GetProjection(const common::PointENU &point_enu,
                                  common::SLPoint *sl_point,
                                  LaneWaypoint *waypoint) const {
  return GetProjection({point_enu.x(), point_enu.y()}, sl_point, waypoint);
}

bool RouteSegments::GetProjection(const common::math::Vec2d &point,
                                  common::SLPoint *sl_point,
                                  LaneWaypoint *waypoint) const {
  double min_l = std::numeric_limits<double>::infinity();
  // accumulated_s记录的是相对于RouteSegments起始点的距离，即相对于RouteSegments的第一个LaneSegment的start_s的距离
  double accumulated_s = 0.0;
  bool has_projection = false;
  // 遍历这个RouteSegments中的每一个LaneSegment
  for (auto iter = begin(); iter != end();
       accumulated_s += (iter->end_s - iter->start_s), ++iter) {
    double lane_s =
        0.0; // 相对于这个LaneSegment所在的那个Lane的起点的距离,不是相对于当前这个LaneSegment的start_s的距离
    double lane_l = 0.0;
    // 从这里来看，一个RouteSegment中的所有的LaneSegment所属的Lane是同一个，这与routing::Passage是能够对应起来的，因为一个Passage就是一个车道
    if (!iter->lane->GetProjection(point, &lane_s, &lane_l)) {
      AERROR << "Failed to get projection from point " << point.DebugString()
             << " on lane " << iter->lane->id().id();
      return false;
    }

    // 满足判断条件说明point不在当前这个LaneSegment上
    if (lane_s < iter->start_s - kSegmentationEpsilon ||
        lane_s > iter->end_s + kSegmentationEpsilon) {
      continue;
    }
    if (std::fabs(lane_l) < min_l) {
      has_projection = true;
      lane_s = std::max(iter->start_s, lane_s);
      lane_s = std::min(iter->end_s, lane_s);
      min_l = std::fabs(lane_l);
      sl_point->set_l(lane_l);
      sl_point->set_s(lane_s - iter->start_s + accumulated_s);
      waypoint->lane = iter->lane;
      waypoint->s =
          lane_s; // 表示的是当前这个waypoint相对于它所在的那个Lane的距离
    }
  }
  return has_projection;
}

bool RouteSegments::GetWaypoint(const double s, LaneWaypoint *waypoint) const {
  double accumulated_s = 0.0;
  bool has_projection = false;
  for (auto iter = begin(); iter != end();
       accumulated_s += (iter->end_s - iter->start_s), ++iter) {
    if (accumulated_s - kSegmentationEpsilon < s &&
        s < accumulated_s + iter->end_s - iter->start_s +
                kSegmentationEpsilon) {
      waypoint->lane = iter->lane;
      waypoint->s = s - accumulated_s + iter->start_s;
      if (waypoint->s < iter->start_s) {
        waypoint->s = iter->start_s;
      } else if (waypoint->s > iter->end_s) {
        waypoint->s = iter->end_s;
      }
      has_projection = true;
      break;
    }
  }
  return has_projection;
}

} // namespace hdmap

} // namespace dharma
