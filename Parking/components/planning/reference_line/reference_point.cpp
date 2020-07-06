#include "planning/reference_line/reference_point.h"

#include "common/util/point_factory.h"

namespace planning {

ReferencePoint::ReferencePoint(const MapPathPoint& map_path_point,
                               const double kappa,
                               const double dkappa)
    : map::MapPathPoint(map_path_point),
      kappa_(kappa),
      dkappa_(dkappa){}

const double& ReferencePoint::kappa() const {
    return kappa_;
}
const double& ReferencePoint::dkappa() const {
    return dkappa_;
}

common::PathPoint ReferencePoint::ToPathPoint(double s) const{
    return common::util::PointFactory::ToPathPoint(x(), y(), s, theta(),
                                                   kappa_, dkappa_);
}

}
