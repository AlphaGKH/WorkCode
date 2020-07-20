#pragma once

#include <string>
#include <vector>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/map/pnc_map/path.h"

namespace dharma {

namespace hdmap {

class RouteSegments : public std::vector<LaneSegment> {
public:
  RouteSegments() = default;

public:
  /**
   * Project a point to this route segment.
   * @param point_enu a map point, or point, which is a Vec2d point
   * @param s return the longitudinal s relative to the route segment.
   * @param l return the lateral distance relative to the route segment.
   * @param waypoint return the LaneWaypoint, which has lane and lane_s on the
   * route segment.
   * @return false if error happened or projected outside of the lane segments.
   */
  bool GetProjection(const common::PointENU &point_enu,
                     common::SLPoint *sl_point, LaneWaypoint *waypoint) const;
  bool GetProjection(const common::math::Vec2d &point,
                     common::SLPoint *sl_point, LaneWaypoint *waypoint) const;

  bool GetWaypoint(const double s, LaneWaypoint *waypoint) const;

  static bool WithinLaneSegment(const LaneSegment &lane_segment,
                                const LaneWaypoint &waypoint);

public:
  bool IsOnSegment() const;
  const std::string &Id() const;

private:
  /**
   * Indicates whether the vehicle is on current RouteSegment.
   **/
  bool is_on_segment_ = false;

  std::string id_;
};

} // namespace hdmap

} // namespace dharma
