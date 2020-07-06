#include "planning/reference_line/reference_line.h"

#include "absl/strings/str_join.h"

#include "common/mlog/mlog.h"
#include "common/util/util.h"
#include "common/math/linear_interpolation.h"
#include "common/math/angle.h"

namespace planning {

ReferenceLine::ReferenceLine(const std::vector<ReferencePoint>& reference_points)
    : reference_points_(reference_points),
      map_path_(std::move(std::vector<map::MapPathPoint>(reference_points.begin(),
                                                         reference_points.end()))){
    CHECK_EQ(map_path_.num_points(), reference_points_.size());
}

ReferenceLine::ReferenceLine(const map::MapPath& map_path)
    : map_path_(map_path) {
    for (const auto& point : map_path.points()) {
        reference_points_.emplace_back(point, 0.0, 0.0);
    }
    CHECK_EQ(map_path_.num_points(), reference_points_.size());
}

bool ReferenceLine::XYToSL(const common::math::Vec2d &xy_point, common::SLPoint *const sl_point) const{
    double s = 0.0;
    double l = 0.0;
    if (!map_path_.GetProjection(xy_point, &s, &l)) {
        AERROR << "Cannot get nearest point from path.";
        return false;
    }
    sl_point->set_s(s);
    sl_point->set_l(l);
    return true;
}

bool ReferenceLine::Segment(const double s, const double look_backward, const double look_forward){
    const auto& accumulated_s = map_path_.accumulated_s();

    // inclusive
    auto start_index =
        std::distance(accumulated_s.begin(),
                      std::lower_bound(accumulated_s.begin(), accumulated_s.end(),
                                       s - look_backward));
    // exclusive
    auto end_index =
        std::distance(accumulated_s.begin(),
                      std::upper_bound(accumulated_s.begin(), accumulated_s.end(),
                                       s + look_forward));

    if (end_index - start_index < 2) {
      AERROR << "Too few reference points after shrinking.";
      return false;
    }

    reference_points_ =
        std::vector<ReferencePoint>(reference_points_.begin() + start_index,
                                    reference_points_.begin() + end_index);

    map_path_ = map::MapPath(std::vector<map::MapPathPoint>(
        reference_points_.begin(), reference_points_.end()));
    return true;
}

ReferencePoint ReferenceLine::GetReferencePoint(const double s) const{

    const auto& accumulated_s = map_path_.accumulated_s();
    if (s < accumulated_s.front() - 1e-2) {
        AWARN << "The requested s: " << s << " < 0.";
        return reference_points_.front();
    }
    if (s > accumulated_s.back() + 1e-2) {
        AWARN << "The requested s: " << s
              << " > reference line length: " << accumulated_s.back();
        return reference_points_.back();
    }

    auto interpolate_index = map_path_.GetIndexFromS(s);

    size_t index = interpolate_index.index;
    size_t next_index = index + 1;

    if (next_index >= reference_points_.size()) {
        next_index = reference_points_.size() - 1;
    }

    const auto& p0 = reference_points_[index];
    const auto& p1 = reference_points_[next_index];

    const double s0 = accumulated_s[index];
    const double s1 = accumulated_s[next_index];

    return InterpolateWithMatchedIndex(p0, s0, p1, s1, interpolate_index);
}

size_t ReferenceLine::GetNearestReferenceIndex(const double s) const{
    const auto& accumulated_s = map_path_.accumulated_s();
    if (s < accumulated_s.front() - 1e-2) {
        AWARN << "The requested s: " << s << " < 0.";
        return 0;
    }
    if (s > accumulated_s.back() + 1e-2) {
        AWARN << "The requested s: " << s << " > reference line length "
              << accumulated_s.back();
        return reference_points_.size() - 1;
    }

    auto it_lower =
            std::lower_bound(accumulated_s.begin(), accumulated_s.end(), s);
    return std::distance(accumulated_s.begin(), it_lower);
}

ReferencePoint ReferenceLine::InterpolateWithMatchedIndex(const ReferencePoint& p0,
                                                          const double& s0,
                                                          const ReferencePoint& p1,
                                                          const double& s1,
                                                          const map::InterpolatedIndex& index) const{
    if (std::fabs(s0 - s1) < common::math::kMathEpsilon) {
        return p0;
    }
    double s = s0 + index.offset;
    DCHECK_LE(s0 - 1.0e-6, s) << "s: " << s << " is less than s0 : " << s0;
    DCHECK_LE(s, s1 + 1.0e-6) << "s: " << s << " is larger than s1: " << s1;

    auto map_path_point = map_path_.GetSmoothPoint(index);

    const double kappa = common::math::lerp(p0.kappa(), s0, p1.kappa(), s1, s);
    const double dkappa = common::math::lerp(p0.dkappa(), s0, p1.dkappa(), s1, s);

    return ReferencePoint(map_path_point, kappa, dkappa);

}

void ReferenceLine::Clear(){
    map_path_.Clear();
    reference_points_.clear();
}

std::string ReferenceLine::DebugString() const {

    return absl::StrCat(
                "point num:", reference_points_.size(),
                absl::StrJoin(reference_points_.begin(),
                              reference_points_.end(), " ",
                              common::util::DebugStringFormatter()));
}


}
