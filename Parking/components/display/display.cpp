#include "display/display.h"

#include "common/mlog/mlog.h"
#include "common/util/file.h"
#include "common/gflags_common.h"

#include "display/common/gflags_display.h"

namespace display {

Display::~Display(){
    if(display_thread_ptr_ && display_thread_ptr_->joinable()){
        display_thread_ptr_->join();
    }
}

bool Display::Init(){
    road_map_ptr_ = std::make_unique<map::RoadMap>();
    global_path_ptr_ = std::make_unique<GlobalPath>(road_map_ptr_.get());

    local_view_ptr_ = std::make_unique<LocalView>();

    if(!common::adapter::AdapterManager::Initialized()){
        if(FLAGS_debug_mode){
            common::adapter::AdapterManager::Init(FLAGS_display_adapter_conf_debug_file);
        }
        else {
            common::adapter::AdapterManager::Init(FLAGS_display_adapter_conf_file);
        }
    }

    if(!common::util::GetProtoFromFile(FLAGS_display_conf_file, &config_)){
        AERROR << "Display unable to parse display config file" << FLAGS_display_conf_file;
        return false;
    }

    if(FLAGS_enable_global_view){
        if(!global_path_ptr_->Init(config_.global_view_config())){
            return false;
        }
    }

    if(FLAGS_enable_local_view){
        if(!local_view_ptr_->Init(config_.local_view_config())){
            return false;
        }
    }

    return true;
}

bool Display::Start(){
    if(!Init()){
        AERROR << "Display has NOT been initiated.";
        return false;
    }

    display_thread_ptr_ = std::make_unique<std::thread>(&Display::DisplayThread,this);

    return true;
}

void Display::DisplayThread(){
    const int32_t kDisplaySleepTime = FLAGS_display_cycle; // ms
    while (!is_stop_) {
        std::this_thread::yield();

        std::this_thread::sleep_for(
                    std::chrono::duration<double, std::milli>(kDisplaySleepTime));

        RunOnce();
    }
    return;
}

void Display::RunOnce(){
    common::adapter::AdapterManager::Observe();

    if(common::adapter::AdapterManager::GetChassisAdapter()->Empty()){
        AWARN << "Chassis not ready";
        return;
    }

    // chassis (vehicle_state)
    const auto chassis = common::adapter::AdapterManager::GetChassisAdapter()->GetLatestObserved();
    common::VehicleState adc_state;
    adc_state.set_x(chassis.x);
    adc_state.set_y(chassis.y);
    adc_state.set_theta(chassis.theta);

    if(FLAGS_enable_global_view){
        if(FLAGS_debug_mode){
            if(common::adapter::AdapterManager::GetMapPathAdapter()->Empty()){
                AWARN << "MapPath not ready";
                return;
            }

            if(common::adapter::AdapterManager::GetRefLineAdapter()->Empty()){
                AWARN << "ReferenceLine not ready";
                return;
            }

            // map_path
            const auto map_path = common::adapter::AdapterManager::GetMapPathAdapter()->GetLatestObserved();
            std::vector<map::MapPathPoint> map_points;
            map_points.clear();
            map_points.reserve(map_path.num_points);

            for(const auto& mp : map_path.lcm_map_path){
                map_points.emplace_back(mp.x, mp.y, mp.theta);
            }
            // reference_line
            const auto ref_line = common::adapter::AdapterManager::GetRefLineAdapter()->GetLatestObserved();
            std::vector<planning::ReferencePoint> ref_points;
            ref_points.clear();
            ref_points.reserve(ref_line.num_points);
            for (const auto& rp : ref_line.lcm_reference_line){
                ref_points.emplace_back(map::MapPathPoint(rp.x, rp.y, rp.theta), 0.0, 0.0);
            }

            global_path_ptr_->Show(adc_state, map_points, ref_points);
        }
        else {
            global_path_ptr_->Show(adc_state);

        }
    }

    if(FLAGS_enable_local_view){
        if(common::adapter::AdapterManager::GetGridMapAdapter()->Empty()){
            AWARN << "GridMap not ready";
            return;
        }

        const auto grid_map = common::adapter::AdapterManager::GetGridMapAdapter()->GetLatestObserved();

        local_view_ptr_->Show(grid_map);
    }

    return;
}

void Display::Stop(){
    AINFO << "Display Stop is Called!";

    is_stop_ = true;
    if(display_thread_ptr_&&display_thread_ptr_->joinable()){
        display_thread_ptr_->join();
    }
}

}
