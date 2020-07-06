#ifndef PLANNING_REFERENCE_LINE_REFERENCE_LINE_H_
#define PLANNING_REFERENCE_LINE_REFERENCE_LINE_H_

#include "common/proto/pnc_point.pb.h"

#include "planning/reference_line/reference_point.h"

namespace planning {

class ReferenceLine
{
public:
    ReferenceLine() = default;
    explicit ReferenceLine(const ReferenceLine& reference_line) = default;
    explicit ReferenceLine(const std::vector<ReferencePoint>& reference_points);
    explicit ReferenceLine(const map::MapPath& map_path);

    const std::vector<ReferencePoint>& reference_points() const{
        return reference_points_;
    }

    const map::MapPath& map_path() const {return map_path_;}

public:
    const double& Length() const {return map_path_.length();}

    void Clear();

    std::string DebugString() const;

public:
    bool XYToSL(const common::math::Vec2d& xy_point, common::SLPoint* const sl_point) const;

    bool Segment(const double s, const double look_backward, const double look_forward);

    ReferencePoint GetReferencePoint(const double s) const;

    size_t GetNearestReferenceIndex(const double s) const;

private:
    ReferencePoint InterpolateWithMatchedIndex(const ReferencePoint& p0,const double& s0,
                                               const ReferencePoint& p1, const double& s1,
                                               const map::InterpolatedIndex& index) const;

private:
    std::vector<ReferencePoint> reference_points_;

    map::MapPath map_path_;

};

}

#endif
