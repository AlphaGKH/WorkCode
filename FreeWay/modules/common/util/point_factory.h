#pragma once

#include "modules/common/proto/pnc_point.pb.h"

namespace dharma {

namespace common {

namespace util {

class PointFactory
{
public:
    static inline PathPoint ToPathPoint(const double x, const double y,
                                        const double z = 0, const double s = 0,
                                        const double theta = 0,
                                        const double kappa = 0,
                                        const double dkappa = 0,
                                        const double ddkappa = 0) {
        PathPoint path_point;
        path_point.set_x(x);
        path_point.set_y(y);
        path_point.set_z(z);
        path_point.set_s(s);
        path_point.set_theta(theta);
        path_point.set_kappa(kappa);
        path_point.set_dkappa(dkappa);
        path_point.set_ddkappa(ddkappa);
        return path_point;
    }
};

}

}

}
