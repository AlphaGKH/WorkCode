#pragma once

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/map/pnc_map/path.h"

namespace dharma {

namespace planning {

class ReferencePoint : public hdmap::MapPathPoint {
public:
  ReferencePoint() = default;

  ReferencePoint(const MapPathPoint &map_path_point, const double kappa,
                 const double dkappa);

public:
  double kappa() const;
  double dkappa() const;

  std::string DebugString() const;

  static void RemoveDuplicates(std::vector<ReferencePoint> *points);

private:
  double kappa_ = 0.0;
  double dkappa_ = 0.0;
};

} // namespace planning

} // namespace dharma
