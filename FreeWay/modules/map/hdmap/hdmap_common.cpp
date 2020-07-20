#include "modules/map/hdmap/hdmap_common.h"

#include "spider/common/log.h"

namespace dharma {

namespace hdmap {

namespace {

// Minimum distance to remove duplicated points.
constexpr double kDuplicatedPointsEpsilon = 1e-7;

void RemoveDuplicates(std::vector<common::math::Vec2d> *points) {
  RETURN_IF_NULL(points);

  int count = 0;
  const double limit = kDuplicatedPointsEpsilon * kDuplicatedPointsEpsilon;
  for (const auto &point : *points) {
    if (count == 0 || point.DistanceSquareTo((*points)[count - 1]) > limit) {
      (*points)[count++] = point;
    }
  }
  points->resize(count);
}

void PointsFromCurve(const Curve &input_curve,
                     std::vector<common::math::Vec2d> *points) {
  RETURN_IF_NULL(points);
  points->clear();

  for (const auto &curve : input_curve.line_segments()) {
    //        if (curve.has_line_segment()) {
    //            for (const auto &point : curve.line_segment().point()) {
    //                CHECK(IsPointValid(point))
    //                        << "invalid map point: " << point.DebugString();
    //                points->emplace_back(point.x(), point.y());
    //            }
    //        } else {
    //            AERROR << "Can not handle curve type.";
    //        }
  }
  RemoveDuplicates(points);
}

} // namespace

LaneInfo::LaneInfo(const Lane &lane) : lane_(lane) { Init(); }

void LaneInfo::GetWidth(const double s, double *left_width,
                        double *right_width) const {
  if (left_width != nullptr) {
    *left_width = GetWidthFromSample(sampled_left_width_, s);
  }
  if (right_width != nullptr) {
    *right_width = GetWidthFromSample(sampled_right_width_, s);
  }
}

void LaneInfo::GetRoadWidth(const double s, double *left_width,
                            double *right_width) const {
  if (left_width != nullptr) {
    *left_width = GetWidthFromSample(sampled_left_road_width_, s);
  }
  if (right_width != nullptr) {
    *right_width = GetWidthFromSample(sampled_right_road_width_, s);
  }
}

void LaneInfo::Init() {
  PointsFromCurve(lane_.central_curve(), &points_);
  CHECK_GE(points_.size(), 2);
  segments_.clear();
  accumulated_s_.clear();
  unit_directions_.clear();
  headings_.clear();

  double s = 0;
  for (size_t i = 0; i + 1 < points_.size(); ++i) {
    segments_.emplace_back(points_[i], points_[i + 1]);
    accumulated_s_.push_back(s);
    unit_directions_.push_back(segments_.back().unit_direction());
    s += segments_.back().length();
  }

  accumulated_s_.push_back(s);
  total_length_ = s;
  CHECK(!unit_directions_.empty());
  unit_directions_.push_back(unit_directions_.back());
  for (const auto &direction : unit_directions_) {
    headings_.push_back(direction.Angle());
  }
  //    for (const auto &overlap_id : lane_.overlap_id()) {
  //        overlap_ids_.emplace_back(overlap_id.id());
  //    }
  CHECK(!segments_.empty());

  sampled_left_width_.clear();
  sampled_right_width_.clear();
  for (const auto &sample : lane_.left_samples()) {
    sampled_left_width_.emplace_back(sample.s(), sample.width());
  }
  for (const auto &sample : lane_.right_samples()) {
    sampled_right_width_.emplace_back(sample.s(), sample.width());
  }

  /*
  if (lane_.has) {
      if (lane_.type() == Lane::CITY_DRIVING) {
          for (const auto &p : sampled_left_width_) {
              if (p.second < FLAGS_half_vehicle_width) {
                  AERROR
                          << "lane[id = " << lane_.id().DebugString()
                          << "]. sampled_left_width_[" << p.second
                          << "] is too small. It should be larger than half
  vehicle width["
                          << FLAGS_half_vehicle_width << "].";
              }
          }
          for (const auto &p : sampled_right_width_) {
              if (p.second < FLAGS_half_vehicle_width) {
                  AERROR
                          << "lane[id = " << lane_.id().DebugString()
                          << "]. sampled_right_width_[" << p.second
                          << "] is too small. It should be larger than half
  vehicle width["
                          << FLAGS_half_vehicle_width << "].";
              }
          }
      } else if (lane_.type() == Lane::NONE) {
          AERROR << "lane_[id = " << lane_.id().DebugString() << "] type is
  NONE.";
      }
  }
  else {
      AERROR << "lane_[id = " << lane_.id().DebugString() << "] has NO type.";
  }

  */

  sampled_left_road_width_.clear();
  sampled_right_road_width_.clear();
  for (const auto &sample : lane_.left_road_samples()) {
    sampled_left_road_width_.emplace_back(sample.s(), sample.width());
  }
  for (const auto &sample : lane_.right_road_samples()) {
    sampled_right_road_width_.emplace_back(sample.s(), sample.width());
  }
}

double
LaneInfo::GetWidthFromSample(const std::vector<LaneInfo::SampledWidth> &samples,
                             const double s) const {
  if (samples.empty()) {
    return 0.0;
  }
  if (s <= samples[0].first) {
    return samples[0].second;
  }
  if (s >= samples.back().first) {
    return samples.back().second;
  }
  int low = 0;
  int high = static_cast<int>(samples.size());
  while (low + 1 < high) {
    const int mid = (low + high) / 2;
    if (samples[mid].first <= s) {
      low = mid;
    } else {
      high = mid;
    }
  }
  const LaneInfo::SampledWidth &sample1 = samples[low];
  const LaneInfo::SampledWidth &sample2 = samples[high];
  const double ratio = (sample2.first - s) / (sample2.first - sample1.first);
  return sample1.second * ratio + sample2.second * (1.0 - ratio);
}

bool LaneInfo::GetProjection(const common::math::Vec2d &point,
                             double *accumulate_s, double *lateral) const {
  RETURN_VAL_IF_NULL(accumulate_s, false);
  RETURN_VAL_IF_NULL(lateral, false);

  if (segments_.empty()) {
    return false;
  }
  double min_dist = std::numeric_limits<double>::infinity();
  int seg_num = static_cast<int>(segments_.size());
  int min_index = 0;
  for (int i = 0; i < seg_num; ++i) {
    const double distance = segments_[i].DistanceSquareTo(point);
    if (distance < min_dist) {
      min_index = i;
      min_dist = distance;
    }
  }
  min_dist = std::sqrt(min_dist);
  const auto &nearest_seg = segments_[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
  } else if (min_index == seg_num - 1) {
    *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
    }
  } else {
    *accumulate_s = accumulated_s_[min_index] +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * min_dist;
  }
  return true;
}

} // namespace hdmap

} // namespace dharma
