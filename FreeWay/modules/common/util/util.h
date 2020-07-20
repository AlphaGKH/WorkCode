/**
 * @projectName : LeapAuto
 * @brief       :
 * @author      : guokonghui
 * @date        : 2019-10-11
 */

#pragma once

#include <cmath>
#include <stdint.h>
#include <string>
#include <vector>

namespace dharma {
namespace common {
namespace util {

template <typename U, typename V> double DistanceXY(const U &u, const V &v) {
  return std::hypot(u.x() - v.x(), u.y() - v.y());
}

} // namespace util
} // namespace common

} // namespace dharma
