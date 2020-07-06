#ifndef PLANNING_MATH_CURVE1D_CURVE1D_H_
#define PLANNING_MATH_CURVE1D_CURVE1D_H_

#include <string>

namespace planning {

class Curve1d {
public:
    Curve1d() = default;

    virtual ~Curve1d() = default;

    virtual double Evaluate(const std::uint32_t order,
                            const double param) const = 0;

    virtual double ParamLength() const = 0;

    virtual std::string ToString() const = 0;
};

}


#endif
