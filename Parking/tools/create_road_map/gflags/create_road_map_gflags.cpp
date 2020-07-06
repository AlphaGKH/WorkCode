#include "create_road_map_gflags.h"

namespace tools {


DEFINE_string(created_road_map_config_file,
              "../create_road_map/conf/created_road_map_config.pb.txt",
              "The configuration file for created_road_map");

DEFINE_string(created_road_map_adapter_conf_file, "../create_road_map/conf/adapter.conf",
              "adapter of created_road_map configure file name");

DEFINE_string(road_map_out_file, "../../data/created_road_map.pb.txt",
              "created_road_map out file name");

}
