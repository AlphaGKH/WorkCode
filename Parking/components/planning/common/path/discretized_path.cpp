#include "planning/common/path/discretized_path.h"

#include "common/mlog/mlog.h"
#include "common/math/linear_interpolation.h"

namespace planning {

DiscretizedPath::DiscretizedPath(const std::vector<common::PathPoint> &path_points)
    : std::vector<common::PathPoint>(std::move(path_points)) {}

double DiscretizedPath::Length() const{
    if (empty()) {
        return 0.0;
    }
    return back().s() - front().s();
}

common::PathPoint DiscretizedPath::Evaluate(const double path_s) const {
    CHECK(!empty());
    auto it_lower = QueryLowerBound(path_s);
    if (it_lower == begin()) {
        return front();
    }
    if (it_lower == end()) {
        return back();
    }

    return common::math::InterpolateUsingLinearApproximation(*(it_lower - 1),
                                                             *it_lower, path_s);


}

std::vector<common::PathPoint>::const_iterator DiscretizedPath::QueryLowerBound(
        const double path_s) const {
    auto func = [](const common::PathPoint &tp, const double path_s) {
        return tp.s() < path_s;
    };
    return std::lower_bound(begin(), end(), path_s, func);
}



}
