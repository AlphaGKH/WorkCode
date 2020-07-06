#ifndef PERCEPTION_OGM_PERCEPTION_OGM_PERCEPTION_H_
#define PERCEPTION_OGM_PERCEPTION_OGM_PERCEPTION_H_

#include <vector>
#include "common/mlog/mlog.h"

#include "perception/proto/ogm_config.pb.h"
#include "perception/common/gflags_perception.h"
#include "perception/Lidar2D.hpp"
#include "perception/GridMap.hpp"

namespace perception {

class OgmPerception
{
public:
    OgmPerception() = default;

    bool Init(const OgmConfig& config);

    bool Process(const Lidar2D& lidar2d, GridMap *grid_map);

private:
    bool SimpleOGM(const Lidar2D& lidar2d, GridMap *grid_map);

    bool FilledOGM(const Lidar2D& lidar2d, GridMap *grid_map);

    bool ExpandedOGM(const Lidar2D& lidar2d, GridMap *grid_map);

private:
    int32_t rows_ = 0;
    int32_t cols_ = 0;
    double angle_resolution_ = 0.25;

private:
    OgmConfig ogm_config_;

};

}

#endif
