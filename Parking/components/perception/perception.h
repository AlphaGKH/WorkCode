#ifndef PERCEPTION_PERCEPTION_H_
#define PERCEPTION_PERCEPTION_H_

#include <thread>

#include "perception/Lidar2D.hpp"

#include "perception/ogm_perception/ogm_perception.h"
#include "perception/proto/perception_config.pb.h"

namespace perception
{

class Perception
{
public:
    Perception() = default;
    ~Perception();

    bool Start();

    void Stop();

private:
    bool Init();

    void PerceptionThread();

    void RunOnce();

    bool PerceptionProcess(const Lidar2D& lidar2d_msg);

private:

    std::unique_ptr<OgmPerception> ogm_perception_;

private:
    bool is_stop_ = false;

    std::unique_ptr<std::thread> perception_thread_ptr_;

private:
    PerceptionConfig config_;



};

} // namespace perception

#endif
