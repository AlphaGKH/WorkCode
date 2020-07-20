#include "modules/map/pnc_map/path.h"

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "gflags/gflags.h"

#include "modules/common/util/string_util.h"

DEFINE_double(default_lane_width, 3.048, "default lane width is about 10 feet");

namespace dharma {

namespace hdmap {

namespace {
const double kSampleDistance = 0.25;
constexpr double kMathEpsilon = 1e-10;

bool FindLaneSegment(const MapPathPoint &p1, const MapPathPoint &p2,
                     LaneSegment *const lane_segment) {
  for (const auto &wp1 : p1.lane_waypoints()) {
    for (const auto &wp2 : p2.lane_waypoints()) {
      if (wp1.lane->id().id() == wp2.lane->id().id() && wp1.s < wp2.s) {
        *lane_segment = LaneSegment(wp1.lane, wp1.s, wp2.s);
        return true;
      }
    }
  }
  return false;
}

} // namespace

// LaneWaypoint
std::string LaneWaypoint::DebugString() const {
  if (lane == nullptr) {
    return "(lane is null)";
  }
  return absl::StrCat("id = ", lane->id().id(), "  s = ", s);
}

// LaneSegment
void LaneSegment::Join(std::vector<LaneSegment> *segments) {
  static constexpr double kSegmentDelta = 0.5;
  std::size_t k = 0;
  std::size_t i = 0;
  while (i < segments->size()) {
    std::size_t j = i;
    while (j + 1 < segments->size() &&
           segments->at(i).lane == segments->at(j + 1).lane) {
      ++j;
    }
    auto &segment_k = segments->at(k);
    segment_k.lane = segments->at(i).lane;
    segment_k.start_s = segments->at(i).start_s;
    segment_k.end_s = segments->at(j).end_s;
    if (segment_k.start_s < kSegmentDelta) {
      segment_k.start_s = 0.0;
    }
    if (segment_k.end_s + kSegmentDelta >= segment_k.lane->total_length()) {
      segment_k.end_s = segment_k.lane->total_length();
    }
    i = j + 1;
    ++k;
  }
  segments->resize(k);
  segments->shrink_to_fit(); // release memory
}

std::string LaneSegment::DebugString() const {
  if (lane == nullptr) {
    return "(lane is null)";
  }
  return absl::StrCat("id = ", lane->id().id(), "  start_s = ", start_s,
                      "  end_s = ", end_s);
}

// MapPathPoint
std::vector<MapPathPoint> MapPathPoint::GetPointsFromLane(LaneInfoConstPtr lane,
                                                          const double start_s,
                                                          const double end_s) {
  std::vector<MapPathPoint> points;
  if (start_s >= end_s) {
    return points;
  }
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {
      points.emplace_back(lane->points()[i], lane->headings()[i],
                          LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto &segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();
      if (start_s > accumulate_s && start_s < next_accumulate_s) {
        points.emplace_back(segment.start() + segment.unit_direction() *
                                                  (start_s - accumulate_s),
                            lane->headings()[i], LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) {
        points.emplace_back(segment.start() + segment.unit_direction() *
                                                  (end_s - accumulate_s),
                            lane->headings()[i], LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
  return points;
}

void MapPathPoint::RemoveDuplicates(std::vector<MapPathPoint> *points) {
  static constexpr double kDuplicatedPointsEpsilon = 1e-7;
  static constexpr double limit =
      kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  CHECK_NOTNULL(points);
  int count = 0;
  for (size_t i = 0; i < points->size(); ++i) {
    if (count == 0 ||
        (*points)[i].DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = (*points)[i];
    } else {
      (*points)[count - 1].add_lane_waypoints((*points)[i].lane_waypoints());
    }
  }
  points->resize(count);
}

std::string MapPathPoint::DebugString() const {
  return absl::StrCat(
      "x = ", x_, "  y = ", y_, "  heading = ", heading_,
      "  lwp = "
      "{(",
      absl::StrJoin(lane_waypoints_, "), (",
                    dharma::common::util::DebugStringFormatter()),
      ")}");
}

// Path

Path::Path(const std::vector<MapPathPoint> &path_points)
    : path_points_(path_points) {
  Init();
}

Path::Path(std::vector<MapPathPoint> &&path_points)
    : path_points_(std::move(path_points)) {
  Init();
}

Path::Path(const std::vector<LaneSegment> &segments)
    : lane_segments_(segments) {
  for (const auto &segment : lane_segments_) {
    const auto points = MapPathPoint::GetPointsFromLane(
        segment.lane, segment.start_s, segment.end_s);
    path_points_.insert(path_points_.end(), points.begin(), points.end());
  }
  MapPathPoint::RemoveDuplicates(&path_points_);
  CHECK_GE(path_points_.size(), 2);
  Init();
}

Path::Path(std::vector<LaneSegment> &&segments)
    : lane_segments_(std::move(segments)) {
  for (const auto &segment : lane_segments_) {
    const auto points = MapPathPoint::GetPointsFromLane(
        segment.lane, segment.start_s, segment.end_s);
    path_points_.insert(path_points_.end(), points.begin(), points.end());
  }
  MapPathPoint::RemoveDuplicates(&path_points_);
  CHECK_GE(path_points_.size(), 2);
  Init();
}

Path::Path(const std::vector<MapPathPoint> &path_points,
           const std::vector<LaneSegment> &lane_segments)
    : path_points_(path_points), lane_segments_(lane_segments) {
  Init();
}

Path::Path(std::vector<MapPathPoint> &&path_points,
           std::vector<LaneSegment> &&lane_segments)
    : path_points_(std::move(path_points)),
      lane_segments_(std::move(lane_segments)) {
  Init();
}

void Path::Init() {
  InitPoints();
  InitLaneSegments();
  InitPointIndex();
  InitWidth();
}

void Path::InitPoints() {
  num_points_ = static_cast<int>(path_points_.size());
  CHECK_GE(num_points_, 2);

  accumulated_s_.clear();
  accumulated_s_.reserve(num_points_);
  segments_.clear();
  segments_.reserve(num_points_);
  unit_directions_.clear();
  unit_directions_.reserve(num_points_);

  double s = 0.0;
  for (int i = 0; i < num_points_; ++i) {
    accumulated_s_.push_back(s);
    common::math::Vec2d heading;
    if (i + 1 >= num_points_) {
      heading = path_points_[i] - path_points_[i - 1];
    } else {
      segments_.emplace_back(path_points_[i], path_points_[i + 1]);
      heading = path_points_[i + 1] - path_points_[i];
      // TODO(All): use heading.length when all adjacent lanes are guarantee to
      // be connected.
      s += heading.Length();
    }
    heading.Normalize();
    unit_directions_.push_back(heading);
  }
  length_ = s;
  num_sample_points_ = static_cast<int>(length_ / kSampleDistance) + 1;
  num_segments_ = num_points_ - 1;

  CHECK_EQ(accumulated_s_.size(), num_points_);
  CHECK_EQ(unit_directions_.size(), num_points_);
  CHECK_EQ(segments_.size(), num_segments_);
}

void Path::InitLaneSegments() {
  if (lane_segments_.empty()) {
    for (int i = 0; i + 1 < num_points_; ++i) {
      LaneSegment lane_segment;
      if (FindLaneSegment(path_points_[i], path_points_[i + 1],
                          &lane_segment)) {
        lane_segments_.push_back(lane_segment);
      }
    }
  }
  LaneSegment::Join(&lane_segments_);
  if (lane_segments_.empty()) {
    return;
  }
  lane_accumulated_s_.resize(lane_segments_.size());
  lane_accumulated_s_[0] = lane_segments_[0].Length();
  for (std::size_t i = 1; i < lane_segments_.size(); ++i) {
    lane_accumulated_s_[i] =
        lane_accumulated_s_[i - 1] + lane_segments_[i].Length();
  }

  lane_segments_to_next_point_.clear();
  lane_segments_to_next_point_.reserve(num_points_);
  for (int i = 0; i + 1 < num_points_; ++i) {
    LaneSegment lane_segment;
    if (FindLaneSegment(path_points_[i], path_points_[i + 1], &lane_segment)) {
      lane_segments_to_next_point_.push_back(lane_segment);
    } else {
      lane_segments_to_next_point_.push_back(LaneSegment());
    }
  }
  CHECK_EQ(lane_segments_to_next_point_.size(), num_segments_);
}

void Path::InitWidth() {
  lane_left_width_.clear();
  lane_left_width_.reserve(num_sample_points_);
  lane_right_width_.clear();
  lane_right_width_.reserve(num_sample_points_);

  road_left_width_.clear();
  road_left_width_.reserve(num_sample_points_);
  road_right_width_.clear();
  road_right_width_.reserve(num_sample_points_);

  double s = 0;
  for (int i = 0; i < num_sample_points_; ++i) {
    const MapPathPoint point = GetSmoothPoint(s);
    if (point.lane_waypoints().empty()) {
      lane_left_width_.push_back(FLAGS_default_lane_width / 2.0);
      lane_right_width_.push_back(FLAGS_default_lane_width / 2.0);

      road_left_width_.push_back(FLAGS_default_lane_width / 2.0);
      road_right_width_.push_back(FLAGS_default_lane_width / 2.0);
      AWARN << "path point:" << point.DebugString() << " has invalid width.";
    } else {
      const LaneWaypoint waypoint = point.lane_waypoints()[0];
      CHECK_NOTNULL(waypoint.lane);

      double lane_left_width = 0.0;
      double lane_right_width = 0.0;
      waypoint.lane->GetWidth(waypoint.s, &lane_left_width, &lane_right_width);
      lane_left_width_.push_back(lane_left_width - waypoint.l);
      lane_right_width_.push_back(lane_right_width + waypoint.l);

      double road_left_width = 0.0;
      double road_right_width = 0.0;
      waypoint.lane->GetRoadWidth(waypoint.s, &road_left_width,
                                  &road_right_width);
      road_left_width_.push_back(road_left_width - waypoint.l);
      road_right_width_.push_back(road_right_width + waypoint.l);
    }
    s += kSampleDistance;
  }
  CHECK_EQ(lane_left_width_.size(), num_sample_points_);
  CHECK_EQ(lane_right_width_.size(), num_sample_points_);

  CHECK_EQ(road_left_width_.size(), num_sample_points_);
  CHECK_EQ(road_right_width_.size(), num_sample_points_);
}

void Path::InitPointIndex() {
  last_point_index_.clear();
  last_point_index_.reserve(num_sample_points_);
  double s = 0.0;
  int last_index = 0;
  for (int i = 0; i < num_sample_points_; ++i) {
    while (last_index + 1 < num_points_ &&
           accumulated_s_[last_index + 1] <= s) {
      ++last_index;
    }
    last_point_index_.push_back(last_index);
    s += kSampleDistance;
  }
  CHECK_EQ(last_point_index_.size(), num_sample_points_);
}

bool Path::GetLaneWidth(const double s, double *lane_left_width,
                        double *lane_right_width) const {
  CHECK_NOTNULL(lane_left_width);
  CHECK_NOTNULL(lane_right_width);

  if (s < 0.0 || s > length_) {
    return false;
  }
  *lane_left_width = GetSample(lane_left_width_, s);
  *lane_right_width = GetSample(lane_right_width_, s);
  return true;
}

bool Path::GetProjection(const common::math::Vec2d &point, double *accumulate_s,
                         double *lateral) const {
  double distance = 0.0;
  return GetProjection(point, accumulate_s, lateral, &distance);
}

bool Path::GetProjection(const common::math::Vec2d &point, double *accumulate_s,
                         double *lateral, double *min_distance) const {
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
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < *min_distance) {
      min_index = i;
      *min_distance = distance;
    }
  }
  *min_distance = std::sqrt(*min_distance);
  const auto &nearest_seg = segments_[min_index];
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

MapPathPoint Path::GetSmoothPoint(const InterpolatedIndex &index) const {
  CHECK_GE(index.id, 0);
  CHECK_LT(index.id, num_points_);

  const MapPathPoint &ref_point = path_points_[index.id];
  if (std::abs(index.offset) > kMathEpsilon) {
    const common::math::Vec2d delta = unit_directions_[index.id] * index.offset;
    MapPathPoint point({ref_point.x() + delta.x(), ref_point.y() + delta.y()},
                       ref_point.heading());
    if (index.id < num_segments_ && !ref_point.lane_waypoints().empty()) {
      const LaneSegment &lane_segment = lane_segments_to_next_point_[index.id];
      auto ref_lane_waypoint = ref_point.lane_waypoints()[0];
      if (lane_segment.lane != nullptr) {
        for (const auto &lane_waypoint : ref_point.lane_waypoints()) {
          if (lane_waypoint.lane->id().id() == lane_segment.lane->id().id()) {
            ref_lane_waypoint = lane_waypoint;
            break;
          }
        }
        point.add_lane_waypoint(
            LaneWaypoint(lane_segment.lane, lane_segment.start_s + index.offset,
                         ref_lane_waypoint.l));
      }
    }
    if (point.lane_waypoints().empty() && !ref_point.lane_waypoints().empty()) {
      point.add_lane_waypoint(ref_point.lane_waypoints()[0]);
    }
    return point;
  } else {
    return ref_point;
  }
}

MapPathPoint Path::GetSmoothPoint(double s) const {
  return GetSmoothPoint(GetIndexFromS(s));
}

InterpolatedIndex Path::GetIndexFromS(double s) const {
  if (s <= 0.0) {
    return {0, 0.0};
  }
  CHECK_GT(num_points_, 0);
  if (s >= length_) {
    return {num_points_ - 1, 0.0};
  }
  const int sample_id = static_cast<int>(s / kSampleDistance);
  if (sample_id >= num_sample_points_) {
    return {num_points_ - 1, 0.0};
  }
  const int next_sample_id = sample_id + 1;
  int low = last_point_index_[sample_id];
  int high = (next_sample_id < num_sample_points_
                  ? std::min(num_points_, last_point_index_[next_sample_id] + 1)
                  : num_points_);
  while (low + 1 < high) {
    const int mid = (low + high) / 2;
    if (accumulated_s_[mid] <= s) {
      low = mid;
    } else {
      high = mid;
    }
  }
  return {low, s - accumulated_s_[low]};
}

double Path::GetSample(const std::vector<double> &samples,
                       const double s) const {
  if (samples.empty()) {
    return 0.0;
  }
  if (s <= 0.0) {
    return samples[0];
  }
  const int idx = static_cast<int>(s / kSampleDistance);
  if (idx >= num_sample_points_ - 1) {
    return samples.back();
  }
  const double ratio = (s - idx * kSampleDistance) / kSampleDistance;
  return samples[idx] * (1.0 - ratio) + samples[idx + 1] * ratio;
}

} // namespace hdmap

} // namespace dharma
