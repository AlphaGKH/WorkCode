#ifndef PLANNING_COMMON_SPEED_SPEED_DATA_H_
#define PLANNING_COMMON_SPEED_SPEED_DATA_H_

#include <vector>
#include "common/proto/pnc_point.pb.h"

namespace planning {

class SpeedData : public std::vector<common::SpeedPoint>
{
public:
    SpeedData() = default;

    explicit SpeedData(std::vector<common::SpeedPoint> speed_points);

    virtual ~SpeedData() = default;

public:

    void AppendSpeedPoint(const double s, const double time, const double v,
                          const double a, const double da);

    bool EvaluateByTime(const double time,
                        common::SpeedPoint* const speed_point) const;

    double TotalTime() const;

};

}


#endif
