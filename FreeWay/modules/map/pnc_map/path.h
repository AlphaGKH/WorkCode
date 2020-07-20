#pragma once

#include "absl/strings/str_cat.h"

#include "spider/common/log.h"

#include "modules/common/math/vec2d.h"

#include "modules/map/hdmap/hdmap_common.h"

namespace dharma {

namespace hdmap {

// 1. LaneWaypoint
struct LaneWaypoint {
  LaneWaypoint() = default;

  LaneWaypoint(LaneInfoConstPtr lane, const double s)
      : lane(CHECK_NOTNULL(lane)), s(s) {}

  LaneWaypoint(LaneInfoConstPtr lane, const double s, const double l)
      : lane(CHECK_NOTNULL(lane)), s(s), l(l) {}

  LaneInfoConstPtr lane = nullptr;
  double s = 0.0; // 注意这个s指的是以当前的点所在的LaneSegment的起点为零点的
  double l = 0.0;

  std::string DebugString() const;
};

// 2. LaneSegment
struct LaneSegment {
  LaneSegment() = default;
  LaneSegment(LaneInfoConstPtr lane, const double start_s, const double end_s)
      : lane(CHECK_NOTNULL(lane)), start_s(start_s), end_s(end_s) {}

  LaneInfoConstPtr lane = nullptr;
  double start_s = 0.0;
  double end_s = 0.0;
  double Length() const { return end_s - start_s; }

  /**
   * Join neighboring lane segments if they have the same lane id
   */
  static void Join(std::vector<LaneSegment> *segments);

  std::string DebugString() const;
};

// 3. MapPathPoint
class MapPathPoint : public common::math::Vec2d {
public:
  MapPathPoint() = default;

  MapPathPoint(const common::math::Vec2d &point, double heading)
      : Vec2d(point.x(), point.y()), heading_(heading) {}

  MapPathPoint(const common::math::Vec2d &point, double heading,
               LaneWaypoint lane_waypoint)
      : Vec2d(point.x(), point.y()), heading_(heading) {
    lane_waypoints_.emplace_back(std::move(lane_waypoint));
  }

  MapPathPoint(const common::math::Vec2d &point, double heading,
               std::vector<LaneWaypoint> lane_waypoints)
      : Vec2d(point.x(), point.y()), heading_(heading),
        lane_waypoints_(std::move(lane_waypoints)) {}

public:
  double heading() const { return heading_; }

  void set_heading(const double heading) { heading_ = heading; }

  const std::vector<LaneWaypoint> &lane_waypoints() const {
    return lane_waypoints_;
  }

  void add_lane_waypoint(LaneWaypoint lane_waypoint) {
    lane_waypoints_.emplace_back(std::move(lane_waypoint));
  }

  void add_lane_waypoints(const std::vector<LaneWaypoint> &lane_waypoints) {
    lane_waypoints_.insert(lane_waypoints_.end(), lane_waypoints.begin(),
                           lane_waypoints.end());
  }

  void clear_lane_waypoints() { lane_waypoints_.clear(); }

public:
  static std::vector<MapPathPoint> GetPointsFromLane(LaneInfoConstPtr lane,
                                                     const double start_s,
                                                     const double end_s);
  static void RemoveDuplicates(std::vector<MapPathPoint> *points);

  std::string DebugString() const;

protected:
  double heading_ = 0.0;
  std::vector<LaneWaypoint> lane_waypoints_;
};

class InterpolatedIndex {
public:
  InterpolatedIndex(int id, double offset) : id(id), offset(offset) {}
  int id = 0;
  double offset = 0.0;
};

class Path {
public:
  Path() = default;

  explicit Path(const std::vector<MapPathPoint> &path_points);
  explicit Path(std::vector<MapPathPoint> &&path_points);

  explicit Path(std::vector<LaneSegment> &&segments);
  explicit Path(const std::vector<LaneSegment> &segments);

  Path(const std::vector<MapPathPoint> &path_points,
       const std::vector<LaneSegment> &lane_segments);

  Path(std::vector<MapPathPoint> &&path_points,
       std::vector<LaneSegment> &&lane_segments);

public:
  int num_points() const { return num_points_; }

  const std::vector<MapPathPoint> &path_points() const { return path_points_; }

  const std::vector<double> &accumulated_s() const { return accumulated_s_; }

  double length() const { return length_; }

public:
  bool GetLaneWidth(const double s, double *lane_left_width,
                    double *lane_right_width) const;

  bool GetProjection(const common::math::Vec2d &point, double *accumulate_s,
                     double *lateral) const; // get
  bool GetProjection(const common::math::Vec2d &point, double *accumulate_s,
                     double *lateral, double *distance) const;

  // Return smooth coordinate by interpolated index or accumulate_s.
  MapPathPoint GetSmoothPoint(const InterpolatedIndex &index) const;
  MapPathPoint GetSmoothPoint(double s) const;

  InterpolatedIndex GetIndexFromS(double s) const;

protected:
  void Init();
  void InitPoints();
  void InitLaneSegments();
  void InitWidth();
  void InitPointIndex();

  double GetSample(const std::vector<double> &samples, const double s) const;

protected:
  std::vector<MapPathPoint> path_points_;
  int num_points_ = 0;

  std::vector<double> accumulated_s_;
  double length_ = 0.0;
  std::vector<common::math::LineSegment2d> segments_;
  int num_segments_ = 0;

  std::vector<common::math::Vec2d> unit_directions_;

protected:
  std::vector<LaneSegment> lane_segments_;
  std::vector<double> lane_accumulated_s_;
  std::vector<LaneSegment> lane_segments_to_next_point_;

protected:
  // Sampled every fixed length.
  int num_sample_points_ = 0;
  std::vector<double> lane_left_width_;
  std::vector<double> lane_right_width_;

  std::vector<double> road_left_width_;
  std::vector<double> road_right_width_;

  std::vector<int> last_point_index_;
};

} // namespace hdmap

} // namespace dharma
