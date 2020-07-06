#ifndef PLANNING_REFERENCE_LINE_QP_SPLINE_REFERENCE_LINE_SMOOTHER_H_
#define PLANNING_REFERENCE_LINE_QP_SPLINE_REFERENCE_LINE_SMOOTHER_H_

#include <vector>
#include <memory>

#include "planning/reference_line/reference_line_smoother.h"
#include "planning/math/smoothing_spline/spline_2d_solver.h"

namespace planning {

class QpSplineReferenceLineSmoother : public ReferenceLineSmoother {
public:
    explicit QpSplineReferenceLineSmoother(
            const ReferenceLineSmootherConfig& config);

    virtual ~QpSplineReferenceLineSmoother() = default;

    bool Smooth(const ReferenceLine& raw_reference_line,
                ReferenceLine* const smoothed_reference_line) override;

    void SetAnchorPoints(const std::vector<AnchorPoint>& achor_points) override;

private:
    void Clear();

    bool Sampling();

    bool AddConstraint();

    bool AddKernel();

    bool Solve();

    bool ExtractEvaluatedPoints(
            const ReferenceLine& raw_reference_line, const std::vector<double>& vec_t,
            std::vector<common::PathPoint>* const path_points) const;

    bool GetSFromParamT(const double t, double* const s) const;

    std::uint32_t FindIndex(const double t) const;

private:
    std::vector<double> t_knots_;
    std::vector<AnchorPoint> anchor_points_;
    std::unique_ptr<Spline2dSolver> spline_solver_;

    double ref_x_ = 0.0;
    double ref_y_ = 0.0;
};

}

#endif
