#include "common/gflags_common.h"

// common
DEFINE_string(vehicle_param_file, "../components/common/conf/vehicle_param.pb.txt", "file contains vehicle param");

// map
DEFINE_string(road_map_file, "../data/road_map.txt", "road_map file");
DEFINE_string(road_map_pb_file, "../data/created_road_map.pb.txt", "created road_map file");

// for debug info
DEFINE_bool(debug_mode, true, "whether enable debug mode for some compoments"); // use in display



// visual debug
DEFINE_bool(enable_visual_debug, true, "enable a visual window for debug");






