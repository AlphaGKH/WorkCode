#pragma once

#include <vector>

#include "modules/common/math/box2d.h"

#include "modules/planning/proto/sl_boundary.pb.h"
#include "modules/planning/reference_line/reference_point.h"


namespace dharma {

namespace planning {

class ReferenceLine
{
public:
    ReferenceLine() = default;

    explicit ReferenceLine(const ReferenceLine& reference_line) = default;

    explicit ReferenceLine(const std::vector<ReferencePoint>& reference_points);

    template <typename Iterator>
    ReferenceLine(const Iterator begin, const Iterator end)
        : reference_points_(begin, end),
          map_path_(std::move(std::vector<hdmap::MapPathPoint>(begin, end))) {}

    explicit ReferenceLine(const hdmap::Path& hdmap_path);

public:
    bool Segment(const common::math::Vec2d& point, const double distance_backward,
                 const double distance_forward);

    bool Segment(const double s, const double distance_backward,
                 const double distance_forward);

public:
    const std::vector<ReferencePoint>& reference_points() const {
        return reference_points_;
    }

public:
    bool GetLaneWidth(const double s, double* const lane_left_width,
                      double* const lane_right_width) const;

    double GetSpeedLimitFromS(const double s) const;

    ReferencePoint GetReferencePoint(const double s) const;

    ReferencePoint InterpolateWithMatchedIndex(
            const ReferencePoint& p0, const double s0, const ReferencePoint& p1,
            const double s1, const hdmap::InterpolatedIndex& index) const;

    double Length() const { return map_path_.length(); }

public:
    bool GetSLBoundary(const common::math::Box2d& box, SLBoundary* const sl_boundary) const;

    bool XYToSL(const common::math::Vec2d& xy_point, common::SLPoint* const sl_point) const;



private:
    std::vector<ReferencePoint> reference_points_;
    hdmap::Path map_path_;

private:
    struct SpeedLimit {
        double start_s = 0.0;
        double end_s = 0.0;
        double speed_limit = 0.0;  // unit m/s
        SpeedLimit() = default;
        SpeedLimit(double _start_s, double _end_s, double _speed_limit)
            : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
    };
    /**
      * This speed limit overrides the lane speed limit
      **/
    std::vector<SpeedLimit> speed_limit_;
};


}
}
