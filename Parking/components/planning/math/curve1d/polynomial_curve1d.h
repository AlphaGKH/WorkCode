#ifndef PLANNING_MATH_CURVE1D_POLYNOMIAL_CURVE1D_H_
#define PLANNING_MATH_CURVE1D_POLYNOMIAL_CURVE1D_H_

#include "planning/math/curve1d/curve1d.h"

namespace planning {

class PolynomialCurve1d : public Curve1d {
public:
    PolynomialCurve1d() = default;
    virtual ~PolynomialCurve1d() = default;

    virtual double Coef(const size_t order) const = 0;
    virtual size_t Order() const = 0;

protected:
    double param_ = 0.0;
};

}  // namespace planning
#endif
