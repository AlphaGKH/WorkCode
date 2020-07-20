#pragma once

#include "gflags/gflags.h"

DECLARE_double(look_backward_distance);
DECLARE_double(look_forward_short_distance);
DECLARE_double(look_forward_long_distance);

namespace dharma {

namespace hdmap {

class PncMap {
public:
  PncMap() = default;
  virtual ~PncMap() = default;

public:
  static double LookForwardDistance(const double velocity); // get
};

} // namespace hdmap

} // namespace dharma
