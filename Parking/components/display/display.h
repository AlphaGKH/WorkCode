#ifndef DISPLAY_DISPLAY_H_
#define DISPLAY_DISPLAY_H_

#include <thread>

#include "common/adapters/adapter_manager.h"

#include "display/proto/display_config.pb.h"

#include "display/global_path/global_path.h"
#include "display/local_view/local_view.h"

namespace display {

class Display
{
public:
    Display() = default;
    ~Display();

    bool Start();

    void Stop();

private:
    bool Init();

    void DisplayThread();

    void RunOnce();

private:

    std::unique_ptr<GlobalPath> global_path_ptr_;
    std::unique_ptr<map::RoadMap> road_map_ptr_;

    std::unique_ptr<LocalView> local_view_ptr_;


private:
    bool is_stop_ = false;

    std::unique_ptr<std::thread> display_thread_ptr_;

private:
    DisplayConfig config_;
};

}

#endif
