#include "common/adapters/adapter_manager.h"

#include "common/util/file.h"


namespace common {

namespace adapter {

AdapterManager::AdapterManager() {}

AdapterManager::~AdapterManager() {}

void AdapterManager::Init(const std::string &adapter_config_filename){
    AdapterManagerConfig configs;
    CHECK(util::GetProtoFromFile(adapter_config_filename,&configs))
            << "Unable to parse adapter config file " << adapter_config_filename;
    //AINFO << "Init AdapterManger config:" << configs.DebugString();
    Init(configs);

}

void AdapterManager::Init(const AdapterManagerConfig &configs){
    if (Initialized()) {
        return;
    }

    instance()->initialized_ = true;

    for(const auto& config : configs.config()){
        switch (config.type()) {

        case AdapterConfig::LIDAR2D:
            EnableLidar2DAdapter("LIDAR2D",config);
            break;
        case AdapterConfig::GRIDMAP:
            EnableGridMapAdapter("GRIDMAP",config);
            break;
        case AdapterConfig::CHASSIS:
            EnableChassisAdapter("CHASSIS", config);
            break;
        case AdapterConfig::COMMAND:
            EnableCommandAdapter("COMMAND", config);
            break;
        case AdapterConfig::TRAJECTORY:
            EnableTrajectoryAdapter("TRAJECTORY", config);
            break;
        case AdapterConfig::MAPPATH:
            EnableMapPathAdapter("MAPPATH", config);
            break;
        case AdapterConfig::REFLINE:
            EnableRefLineAdapter("REFLINE", config);
            break;
        default:
            AERROR << "Unknown adapter config type!";
            break;

        }
    }

}

bool AdapterManager::Initialized() { return instance()->initialized_; }

void AdapterManager::Observe() {
    for (const auto observe : instance()->observers_) {
        observe();
    }
}

}

}
