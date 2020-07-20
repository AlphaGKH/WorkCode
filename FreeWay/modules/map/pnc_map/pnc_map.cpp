#include "modules/map/pnc_map/pnc_map.h"

#include "spider/common/log.h"

#include "modules/common/configs/config_gflags.h"
#include "modules/common/util/util.h"

DEFINE_double(
    look_backward_distance, 50,
    "look backward this distance when creating reference line from routing");

DEFINE_double(look_forward_short_distance, 180,
              "short look forward this distance when creating reference line "
              "from routing when ADC is slow");
DEFINE_double(
    look_forward_long_distance, 250,
    "look forward this distance when creating reference line from routing");

namespace dharma {

namespace hdmap {

PncMap::PncMap(const HDMap *hdmap) : hdmap_(hdmap) {}

const hdmap::HDMap *PncMap::hdmap() const { return hdmap_; }

bool PncMap::IsNewRouting(const routing::RoutingResponse &routing) const {
  return IsNewRouting(routing_, routing);
}

bool PncMap::IsNewRouting(const routing::RoutingResponse &prev,
                          const routing::RoutingResponse &routing) {
  if (!ValidateRouting(routing)) {
    ADEBUG << "The provided routing is invalid.";
    return false;
  }
  return !common::util::IsProtoEqual(prev, routing);
}

bool PncMap::ValidateRouting(const routing::RoutingResponse &routing) {
  const int num_road = routing.road_size();
  if (num_road == 0) {
    AERROR << "Route is empty.";
    return false;
  }
  if (!routing.has_routing_request() ||
      routing.routing_request().waypoint_size() < 2) {
    AERROR << "Routing does not have request.";
    return false;
  }
  for (const auto &waypoint : routing.routing_request().waypoint()) {
    if (!waypoint.has_id() || !waypoint.has_s()) {
      AERROR << "Routing waypoint has no lane_id or s.";
      return false;
    }
  }
  return true;
}

double PncMap::LookForwardDistance(double velocity) {
  auto forward_distance = velocity * FLAGS_look_forward_time_sec;

  return forward_distance > FLAGS_look_forward_short_distance
             ? FLAGS_look_forward_long_distance
             : FLAGS_look_forward_short_distance;
}

} // namespace hdmap

} // namespace dharma
