#include "common/vehicle_param_handle/vehicle_param_handle.h"
#include <cmath>

#include "common/gflags_common.h"
#include "common/util/file.h"

namespace common {

VehicleParam VehicleParamHandle::vehicle_param_;
bool VehicleParamHandle::is_init_ = false;

VehicleParamHandle::VehicleParamHandle() {}

void VehicleParamHandle::Init(){
    Init(FLAGS_vehicle_param_file);
}

void VehicleParamHandle::Init(const VehicleParam &param){
    vehicle_param_ = param;
    is_init_ = true;
}

void VehicleParamHandle::Init(const std::string &param_file){
    VehicleParam param;
    CHECK(common::util::GetProtoFromFile(param_file, &param))
            << "Unable to parse vehicle config file " << param_file;
    Init(param);
}

const VehicleParam& VehicleParamHandle::GetParam(){
    if(!is_init_){
        Init();
    }

    return vehicle_param_;
}

double VehicleParamHandle::MinSafeTurnRadius(){
    const auto &param = vehicle_param_;
    double lat_edge_to_center =
        std::max(param.left_edge_to_center(), param.right_edge_to_center());
    double lon_edge_to_center =
        std::max(param.front_edge_to_center(), param.back_edge_to_center());
    return std::sqrt((lat_edge_to_center + param.min_turn_radius()) *
                         (lat_edge_to_center + param.min_turn_radius()) +
                     lon_edge_to_center * lon_edge_to_center);

}




}

