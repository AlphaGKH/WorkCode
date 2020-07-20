#include <memory>

#include "modules/map/proto/map_lane.pb.h"

#include "modules/common/math/line_segment2d.h"

namespace dharma {

namespace hdmap {

class LaneInfo;

using LaneInfoConstPtr = std::shared_ptr<const LaneInfo>;

class LaneInfo {
public:
  explicit LaneInfo(const Lane &lane);

public:
  const Id &id() const { return lane_.id(); }

  const Lane &lane() const { return lane_; }

  const std::vector<common::math::Vec2d> &points() const { return points_; }

  const std::vector<double> &headings() const { return headings_; }

  const std::vector<common::math::LineSegment2d> &segments() const {
    return segments_;
  }

  double total_length() const { return total_length_; }

public:
  using SampledWidth = std::pair<double, double>;

  void GetWidth(const double s, double *left_width, double *right_width) const;

  void GetRoadWidth(const double s, double *left_width,
                    double *right_width) const;

  bool GetProjection(const common::math::Vec2d &point, double *accumulate_s,
                     double *lateral) const;

private:
  void Init();

  double GetWidthFromSample(const std::vector<LaneInfo::SampledWidth> &samples,
                            const double s) const;

private:
  const Lane &lane_;

  std::vector<common::math::Vec2d> points_;
  std::vector<common::math::LineSegment2d> segments_;
  std::vector<double> accumulated_s_;
  std::vector<common::math::Vec2d> unit_directions_;
  std::vector<double> headings_;
  double total_length_ = 0.0;

  std::vector<SampledWidth> sampled_left_width_;
  std::vector<SampledWidth> sampled_right_width_;

  std::vector<SampledWidth> sampled_left_road_width_;
  std::vector<SampledWidth> sampled_right_road_width_;
};

} // namespace hdmap

} // namespace dharma
