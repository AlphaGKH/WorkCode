#include "planning/common/speed/speed_data.h"

//#include <algorithm>
//#include <utility>

#include "common/math/linear_interpolation.h"
#include "common/util/point_factory.h"

namespace planning {

//using common::SpeedPoint;

SpeedData::SpeedData(std::vector<common::SpeedPoint> speed_points)
    : std::vector<common::SpeedPoint>(std::move(speed_points)) {
    std::sort(begin(), end(), [](const common::SpeedPoint& p1, const common::SpeedPoint& p2) {
        return p1.t() < p2.t();
    });
}

void SpeedData::AppendSpeedPoint(const double s, const double time,
                                 const double v, const double a,
                                 const double da) {
    if (!empty()) {
        CHECK(back().t() < time);
    }

    push_back(common::util::PointFactory::ToSpeedPoint(s, time, v, a, da));
}

bool SpeedData::EvaluateByTime(const double t,
                               common::SpeedPoint* const speed_point) const {
    if (size() < 2) {
        return false;
    }
    if (!(front().t() < t + 1.0e-6 && t - 1.0e-6 < back().t())) {
        return false;
    }

    auto comp = [](const common::SpeedPoint& sp, const double t) {
        return sp.t() < t;
    };

    auto it_lower =
            std::lower_bound(begin(), end(), t, comp);
    if (it_lower == end()) {
        *speed_point = back();
    } else if (it_lower == begin()) {
        *speed_point = front();
    } else {
        const auto& p0 = *(it_lower - 1);
        const auto& p1 = *it_lower;
        double t0 = p0.t();
        double t1 = p1.t();

        speed_point->Clear();
        speed_point->set_s(common::math::lerp(p0.s(), t0, p1.s(), t1, t));
        speed_point->set_t(t);
        speed_point->set_v(common::math::lerp(p0.v(), t0, p1.v(), t1, t));
        speed_point->set_a(common::math::lerp(p0.a(), t0, p1.a(), t1, t));
        speed_point->set_da(common::math::lerp(p0.da(), t0, p1.da(), t1, t));

    }
    return true;
}

double SpeedData::TotalTime() const {
    if (empty()) {
        return 0.0;
    }
    return back().t() - front().t();
}


}
