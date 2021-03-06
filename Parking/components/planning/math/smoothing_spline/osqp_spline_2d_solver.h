#ifndef PLANNING_MATH_SMOOTHING_SPLINE_OSQP_SPLINE_2D_SOLVER_H_
#define PLANNING_MATH_SMOOTHING_SPLINE_OSQP_SPLINE_2D_SOLVER_H_

#include <vector>

#include "qpOASES.hpp"

#include "planning/math/smoothing_spline/spline_2d.h"
#include "planning/math/smoothing_spline/spline_2d_solver.h"

namespace planning {

class OsqpSpline2dSolver final : public Spline2dSolver {
public:
    OsqpSpline2dSolver(const std::vector<double>& t_knots, const uint32_t order);

    void Reset(const std::vector<double>& t_knots, const uint32_t order) override;

    // customize setup
    Spline2dConstraint* mutable_constraint() override;
    Spline2dKernel* mutable_kernel() override;
    Spline2d* mutable_spline() override;

    // solve
    bool Solve() override;

    // extract
    const Spline2d& spline() const override;

private:
    OSQPSettings* osqp_settings_ = nullptr;
    OSQPWorkspace* work_ = nullptr;  // Workspace
    OSQPData* data_ = nullptr;       // OSQPData

    int last_num_constraint_ = 0;
    int last_num_param_ = 0;
    bool last_problem_success_ = false;
};

}  // namespace planning

#endif
