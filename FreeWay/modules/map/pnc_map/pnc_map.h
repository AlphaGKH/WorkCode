#pragma once

#include "gflags/gflags.h"

#include "modules/map/hdmap/hdmap.h"

#include "modules/routing/proto/routing.pb.h"

DECLARE_double(look_backward_distance);
DECLARE_double(look_forward_short_distance);
DECLARE_double(look_forward_long_distance);

namespace dharma {

namespace hdmap {

class PncMap {
public:
  virtual ~PncMap() = default;
  explicit PncMap(const HDMap *hdmap);

  const HDMap *hdmap() const;

public:
  /**
   * Check if the routing is the same as existing one in PncMap
   */
  bool IsNewRouting(const routing::RoutingResponse &routing_response) const;
  static bool IsNewRouting(const routing::RoutingResponse &prev,
                           const routing::RoutingResponse &routing_response);

  static double LookForwardDistance(const double velocity);

private:
  static bool ValidateRouting(const routing::RoutingResponse &routing);

private:
  const HDMap *hdmap_ = nullptr;

private:
  routing::RoutingResponse routing_;
};

} // namespace hdmap

} // namespace dharma
