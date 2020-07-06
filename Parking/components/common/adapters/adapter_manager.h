#ifndef COMMON_ADAPTERS_ADAPTER_MANAGER_H_
#define COMMON_ADAPTERS_ADAPTER_MANAGER_H_

#include <memory>

#include "common/macro.h"
#include "common/mlog/mlog.h"

#include "lcm/lcm-cpp.hpp"

#include "common/adapters/adapter.h"
#include "common/adapters/proto/adapter_config.pb.h"

#include "common/adapters/message_adapter.h"

namespace common {

namespace adapter {

#define REGISTER_ADAPTER(type)                                                                              \
    public:                                                                                                 \
        static bool Enable##type##Adapter(const std::string& channel_name, const AdapterConfig& config){    \
            CHECK(config.message_history_limit() > 0)                                                       \
                    << "Message history limit must be greater than 0";                                      \
            return instance()->InternalEnable##type(channel_name, config);                                  \
        }                                                                                                   \
        static type##Adapter *Get##type##Adapter() {                                                        \
            return instance()->InternalGet##type##Adapter();                                                \
        }                                                                                                   \
        static AdapterConfig &Get##type##AdapterConfig() {                                                  \
            return instance()->type##_adapter_config_;                                                      \
        }                                                                                                   \
        static void Publish##type(const type##Adapter::DataType& data){                                     \
            instance()->InternalPublish(data);                                                              \
        }                                                                                                   \
    private:                                                                                                \
        std::unique_ptr<type##Adapter> type##_adapter_;                                                     \
        AdapterConfig type##_adapter_config_;                                                               \
    private:                                                                                                \
        bool InternalEnable##type(const std::string& channel_name, const AdapterConfig& config){            \
            type##_adapter_ = std::make_unique<type##Adapter>(channel_name,config.message_history_limit()); \
            if(!type##_adapter_->EnableLcm()){                                                              \
                return false;                                                                               \
            }                                                                                               \
            if(config.mode() != AdapterConfig::PUBLISH_ONLY){                                               \
                type##_adapter_->Subscribe();                                                               \
            }                                                                                               \
            observers_.push_back([this]() { type##_adapter_->Observe(); });                                 \
            type##_adapter_config_ = config;                                                                \
            return true;                                                                                    \
        }                                                                                                   \
        type##Adapter *InternalGet##type##Adapter() { return type##_adapter_.get(); }                       \
        void InternalPublish(const type##Adapter::DataType& data){                                          \
            type##_adapter_->Publish(data);                                                                 \
            type##_adapter_->SetLatestPublished(data);                                                      \
        }

class AdapterManager
{
public:
    static void Init(const std::string& adapter_config_filename);

    static void Init(const AdapterManagerConfig& configs);

    static bool Initialized();

    static void Observe();

private:
    bool initialized_ = false;

    std::vector<std::function<void()>> observers_;
    REGISTER_ADAPTER(Lidar2D)
    REGISTER_ADAPTER(GridMap)
    REGISTER_ADAPTER(Chassis)
    REGISTER_ADAPTER(Command)
    REGISTER_ADAPTER(Trajectory)
    REGISTER_ADAPTER(MapPath)
    REGISTER_ADAPTER(RefLine)

    DECLARE_SINGLETON_ADAPTER(AdapterManager)
};

}

}



#endif
