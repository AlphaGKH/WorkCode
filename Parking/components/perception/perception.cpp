#include "perception/perception.h"

#include "common/mlog/mlog.h"
#include "common/adapters/adapter_manager.h"
#include "common/util/file.h"

#include "perception/common/gflags_perception.h"

namespace perception
{

Perception::~Perception()
{
    if (perception_thread_ptr_ && perception_thread_ptr_->joinable())
    {
        perception_thread_ptr_->join();
    }
}

bool Perception::Init()
{
    if(!common::util::GetProtoFromFile(FLAGS_perception_conf_file, &config_)){
        AERROR << "Perception unable to parse perception config file " << FLAGS_perception_conf_file;
        return false;
    }

    ogm_perception_ = std::make_unique<OgmPerception>();

    if(!ogm_perception_->Init(config_.ogm_config())){
        AERROR << "OgmPerception Init Failed!";
        return false;
    }

    if(!common::adapter::AdapterManager::Initialized()){
        common::adapter::AdapterManager::Init(FLAGS_perception_adapter_conf_file);
    }

    return true;
}

bool Perception::Start()
{
    if (!Init())
    {
        AERROR << "Perception has NOT been initiated.";
        return false;
    }

    perception_thread_ptr_ = std::make_unique<std::thread>(&Perception::PerceptionThread, this);

    return true;
}

void Perception::PerceptionThread()
{
    const int32_t kSleepTime = FLAGS_perception_cycle; // ms
    while (!is_stop_)
    {
        std::this_thread::yield();

        std::this_thread::sleep_for(
                    std::chrono::duration<double, std::milli>(kSleepTime));
        RunOnce();
    }
    return;
}

void Perception::RunOnce()
{
    common::adapter::AdapterManager::Observe();

    if(common::adapter::AdapterManager::GetLidar2DAdapter()->Empty()){
        AINFO << "Lidar2D not ready";
        return;
    }

    const auto& lidar2d_msg = common::adapter::AdapterManager::GetLidar2DAdapter()->GetLatestObserved();

    if(!PerceptionProcess(lidar2d_msg)){
        return;
    }

    return;
}

bool Perception::PerceptionProcess(const Lidar2D& lidar2d_msg)
{
    if (lidar2d_msg.distances.size() < 1)
    {
        return false;
    }

    GridMap gridmap_msg;

    // Lidar2d msg to gridmap
    ogm_perception_->Process(lidar2d_msg,&gridmap_msg);

    common::adapter::AdapterManager::PublishGridMap(gridmap_msg);

    return true;
}

void Perception::Stop()
{
    AINFO << "Perception Stop is Called!";

    is_stop_ = true;
    if (perception_thread_ptr_ && perception_thread_ptr_->joinable())
    {
        perception_thread_ptr_->join();
    }
}

} // namespace perception
