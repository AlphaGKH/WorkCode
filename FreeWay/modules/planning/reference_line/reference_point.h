#pragma once

#include "modules/map/pnc_map/path.h"
#include "modules/common/proto/pnc_point.pb.h"

namespace dharma {

namespace planning {

class ReferencePoint : public hdmap::MapPathPoint {
public:
    ReferencePoint() = default;

    ReferencePoint(const MapPathPoint& map_path_point, const double kappa,
                   const double dkappa);

    common::PathPoint ToPathPoint(double s) const;

    double kappa() const;
    double dkappa() const;

    std::string DebugString() const;

    static void RemoveDuplicates(std::vector<ReferencePoint>* points);

private:
    double kappa_ = 0.0;
    double dkappa_ = 0.0;
};

}

}
