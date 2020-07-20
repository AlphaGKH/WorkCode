#include "modules/planning/speed/st_point.h"

#include "modules/common/util/string_util.h"

namespace dharma {
namespace planning {

using dharma::common::util::StringPrintf;

STPoint::STPoint(const double s, const double t) : Vec2d(t, s) {}

STPoint::STPoint(const common::math::Vec2d& vec2d_point) : Vec2d(vec2d_point) {}

double STPoint::s() const { return y_; }

double STPoint::t() const { return x_; }

void STPoint::set_s(const double s) { y_ = s; }

void STPoint::set_t(const double t) { x_ = t; }

std::string STPoint::DebugString() const {
    return StringPrintf("{ \"s\" : %.6f, \"t\" : %.6f }", s(), t());
}

}  // namespace planning
}  // namespace dharma
