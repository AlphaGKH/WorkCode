#ifndef PLANNING_REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_H_
#define PLANNING_REFERENCE_LINE_REFERENCE_LINE_SMOOTHER_H_

#include <vector>
#include "common/proto/pnc_point.pb.h"

#include "planning/reference_line/reference_line.h"
#include "planning/proto/reference_line_provider_config.pb.h"

namespace planning {

struct AnchorPoint {
    common::PathPoint path_point;
    double lateral_bound = 0.0;
    double longitudinal_bound = 0.0;
    // enforce smoother to strictly follow this reference point
    bool enforced = false;
};

class ReferenceLineSmoother {
public:
    explicit ReferenceLineSmoother(const ReferenceLineSmootherConfig& config)
        : smoother_config_(config) {}

    /**
      * Smoothing constraints
      */
    virtual void SetAnchorPoints(
            const std::vector<AnchorPoint>& achor_points) = 0;

    /**
      * Smooth a given reference line
      */
    virtual bool Smooth(const ReferenceLine&, ReferenceLine* const) = 0;

    virtual ~ReferenceLineSmoother() = default;

protected:
    ReferenceLineSmootherConfig smoother_config_;
};




}


#endif
