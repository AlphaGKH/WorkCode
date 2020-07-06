#include "generate_road_map.h"

#include <cmath>

#include "common/util/file.h"
#include "common/adapters/adapter_manager.h"


#include "gflags/create_road_map_gflags.h"

namespace tools {

GenerateRoadMap::~GenerateRoadMap(){
    if(thread_ptr_ && thread_ptr_->joinable()){
        thread_ptr_->join();
    }
}

bool GenerateRoadMap::Init(){

    CHECK(common::util::GetProtoFromFile(FLAGS_created_road_map_config_file, &config_))
            << "Unable to parse created_map config file "
            << FLAGS_created_road_map_config_file;

    if(!common::adapter::AdapterManager::Initialized()){
        common::adapter::AdapterManager::Init(FLAGS_created_road_map_adapter_conf_file);
    }

    if(common::util::PathExists(FLAGS_road_map_out_file)){
        common::util::DeleteFile(FLAGS_road_map_out_file);
    }

    return true;
}

bool GenerateRoadMap::Start(){
    if(!Init()){
        AERROR << "CreateRoadMap not Init!";
        return false;
    }

    thread_ptr_ = std::make_unique<std::thread>(&GenerateRoadMap::ThreadFunction, this);
    return true;

}

void GenerateRoadMap::Stop(){

    is_stop_ = true;
    if(thread_ptr_ && thread_ptr_->joinable()){
        thread_ptr_->join();
    }


    return;
}


void GenerateRoadMap::ThreadFunction(){
    const int32_t kPlanningSleepTime = 1000; // ms
    while (!is_stop_) {
        std::this_thread::yield();
        std::this_thread::sleep_for(
                    std::chrono::duration<double, std::milli>(kPlanningSleepTime));

        RunOnce();
    }

    return;

}

void GenerateRoadMap::RunOnce(){
    common::adapter::AdapterManager::Observe();

    if(common::adapter::AdapterManager::GetChassisAdapter()->Empty()){
        AINFO << "Chassis not ready";
        return;
    }

    const auto chassis = common::adapter::AdapterManager::GetChassisAdapter()->GetLatestObserved();

    map::CreatedMapPoint point;
    point.set_id(number_);
    point.set_x(chassis.x);
    point.set_y(chassis.y);
    point.set_theta(chassis.theta);
    point.set_lane_left_width(config_.default_lane_left_width());
    point.set_lane_right_width(config_.default_lane_right_width());
    point.set_road_left_width(config_.default_road_left_width());
    point.set_road_right_width(config_.default_road_right_width());
    point.set_lane_number(config_.default_lane_number());

    if(road_map_.points().empty()){
        const auto p = road_map_.add_points();
        p->CopyFrom(point);
        number_ ++;
    }
    else {
        const auto last_p = road_map_.points().rbegin();

        double delta_dis =
                (last_p->x() - point.x()) * (last_p->x() - point.x())
                + (last_p->y() - point.y()) * (last_p->y() - point.y());

        double delta_theta = std::fabs(last_p->theta() - point.theta());

        if((config_.distance_thod() * config_.distance_thod() < delta_dis)||
                config_.theta_thod() < delta_theta){
            const auto p = road_map_.add_points();
            p->CopyFrom(point);
            number_ ++;
        }
    }

    if(road_map_.points_size() > 5){
        common::util::SetProtoToASCIIFileAppend(road_map_, FLAGS_road_map_out_file);
        road_map_.Clear();
    }

    return;
}

}
