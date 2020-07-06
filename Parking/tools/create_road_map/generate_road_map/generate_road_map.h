#ifndef TOOLS_CREATE_MAP_ROAD_GENERATE_ROAD_MAP_GENERATE_ROAD_MAP_H_
#define TOOLS_CREATE_MAP_ROAD_GENERATE_ROAD_MAP_GENERATE_ROAD_MAP_H_

#include <thread>
#include "common/mlog/mlog.h"

#include "map/proto/created_map.pb.h"

namespace tools {

class GenerateRoadMap
{
public:
    GenerateRoadMap() = default;
    ~GenerateRoadMap();

public:
    bool Init();

    bool Start();

    void Stop();

private:
    void ThreadFunction();

    void RunOnce();

    bool CreateMapProcess();

private:
    bool is_stop_ = false;

    int32_t number_ = 0;

    std::unique_ptr<std::thread> thread_ptr_;

    map::CreatedMapConfig config_;

    map::CreatedMap road_map_;





};

}


#endif
