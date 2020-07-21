#include "modules/planning/speed/st_boundary.h"

#include "modules/common/math/math_utils.h"
#include "modules/planning/common/planning_gflags.h"
#include "spider/common/log.h"

namespace dharma {
namespace planning {

using dharma::common::math::LineSegment2d;
using dharma::common::math::Vec2d;

const std::string &STBoundary::id() const { return id_; }

void STBoundary::set_id(const std::string &id) { id_ = id; }

STPoint STBoundary::upper_left_point() const { return upper_left_point_; }

STPoint STBoundary::upper_right_point() const { return upper_right_point_; }

STPoint STBoundary::bottom_left_point() const { return bottom_left_point_; }

STPoint STBoundary::bottom_right_point() const { return bottom_right_point_; }

void STBoundary::set_upper_left_point(STPoint st_point) {
  upper_left_point_ = std::move(st_point);
}

void STBoundary::set_upper_right_point(STPoint st_point) {
  upper_right_point_ = std::move(st_point);
}

void STBoundary::set_bottom_left_point(STPoint st_point) {
  bottom_left_point_ = std::move(st_point);
}

void STBoundary::set_bottom_right_point(STPoint st_point) {
  bottom_right_point_ = std::move(st_point);
}

} // namespace planning
} // namespace dharma
