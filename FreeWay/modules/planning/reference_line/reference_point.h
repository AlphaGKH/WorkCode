#pragma once

#include "modules/common/math/vec2d.h"

namespace dharma {

namespace planning {

class ReferencePoint : public common::math::Vec2d
{
public:
    ReferencePoint() = default;

public:
    double heading() const { return heading_; }

    double kappa() const { return kappa_; }

    double dkappa() const { return dkappa_; }

private:
    double heading_ = 0.0;
    double kappa_ = 0.0;
    double dkappa_ = 0.0;
};

}

}
