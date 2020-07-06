#ifndef COMMON_GFLAGS_COMMON_H_
#define COMMON_GFLAGS_COMMON_H_

#include "gflags/gflags.h"

// common
DECLARE_string(vehicle_param_file);

// map
DECLARE_string(road_map_file);
DECLARE_string(road_map_pb_file);

// for debug info
DECLARE_bool(debug_mode); // use in display

// visual debug
DECLARE_bool(enable_visual_debug);






#endif
