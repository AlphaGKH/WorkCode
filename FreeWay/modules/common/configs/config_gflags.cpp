#include "modules/common/configs/config_gflags.h"

DEFINE_bool(use_navigation_mode, false,
            "Use relative position in navigation mode");

DEFINE_string(vehicle_config_path,
              "../modules/common/data/vehicle_param.pb.txt",
              "the file path of vehicle config file");

DEFINE_double(look_forward_time_sec, 8.0,
              "look forward time times adc speed to calculate this distance "
              "when creating reference line from routing");
