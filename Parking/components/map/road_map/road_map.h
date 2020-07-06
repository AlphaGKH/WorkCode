#ifndef MAP_ROAD_MAP_ROAD_MAP_H_
#define MAP_ROAD_MAP_ROAD_MAP_H_

#include <string>
#include <vector>

#include "map/common/gflags_map.h"
#include "map/map_path/map_path.h"

namespace map {

class RoadMap
{
public:
    RoadMap() = default;

    ~RoadMap(){
        map_path_points_.clear();
    }

public:
    bool Init();

    const std::vector<MapPathPoint>& map_path_points() const {
        return map_path_points_;
    }

private:

    void LoadRoadMap();

    void LoadRoadMapProto();

    bool Revise();

private:
    std::vector<MapPathPoint> map_path_points_;
};

}



#endif
