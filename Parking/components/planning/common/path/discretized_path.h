#ifndef PLANNING_COMMON_PATH_DISCRETIZED_PATH_H_
#define PLANNING_COMMON_PATH_DISCRETIZED_PATH_H_

#include "common/proto/pnc_point.pb.h"

namespace planning {

class DiscretizedPath : public std::vector<common::PathPoint>
{
public:
    DiscretizedPath() = default;

    explicit DiscretizedPath(const std::vector<common::PathPoint>& path_points);

    virtual ~DiscretizedPath() = default;

public:
    common::PathPoint Evaluate(const double path_s) const;

    double Length() const;

private:
    std::vector<common::PathPoint>::const_iterator QueryLowerBound(const double path_s) const;

};

}

#endif
