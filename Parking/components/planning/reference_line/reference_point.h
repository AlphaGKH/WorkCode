#ifndef PLANNING_REFERENCE_LINE_REFERENCE_POINT_H_
#define PLANNING_REFERENCE_LINE_REFERENCE_POINT_H_

#include "map/map_path/map_path.h"
#include "common/proto/pnc_point.pb.h"

namespace planning {

class ReferencePoint : public map::MapPathPoint
{
public:
    ReferencePoint() = default;

    ReferencePoint(const ReferencePoint&) = default;

    ReferencePoint(const MapPathPoint& map_path_point, const double kappa,
                   const double dkappa);

    const double& kappa() const;
    const double& dkappa() const;

    common::PathPoint ToPathPoint(double s) const;


private:
    double kappa_ = 0.0;
    double dkappa_ = 0.0;
};

}

#endif
