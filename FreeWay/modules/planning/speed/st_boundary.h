#pragma once

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "modules/planning/proto/planning.pb.h"

#include "modules/common/math/box2d.h"
#include "modules/common/math/polygon2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/speed/st_point.h"

namespace dharma {
namespace planning {

class STBoundary : public common::math::Polygon2d {
public:
  /** Constructors:
   *   STBoundary must be initialized with a vector of ST-point pairs.
   *   Each pair refers to a time t, with (lower_s, upper_s).
   */
  STBoundary() = default;

  explicit STBoundary(const common::math::Box2d &box) = delete;
  explicit STBoundary(std::vector<common::math::Vec2d> points) = delete;

  /** @brief Default destructor.
   */
  ~STBoundary() = default;

  const std::string &id() const;

  void set_id(const std::string &id);

  // Used by st-optimizer.
  bool IsPointInBoundary(const STPoint &st_point) const;
  STBoundary ExpandByS(const double s) const;
  STBoundary ExpandByT(const double t) const;

  // Used by Lattice planners.
  STPoint upper_left_point() const;
  STPoint upper_right_point() const;
  STPoint bottom_left_point() const;
  STPoint bottom_right_point() const;

  void set_upper_left_point(STPoint st_point);
  void set_upper_right_point(STPoint st_point);
  void set_bottom_left_point(STPoint st_point);
  void set_bottom_right_point(STPoint st_point);

private:
  std::string id_;

  STPoint bottom_left_point_;
  STPoint bottom_right_point_;
  STPoint upper_left_point_;
  STPoint upper_right_point_;
};

} // namespace planning
} // namespace dharma
