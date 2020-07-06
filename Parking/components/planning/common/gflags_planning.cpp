#include "planning/common/gflags_planning.h"

// planning
DEFINE_string(planning_adapter_conf_file, "../components/planning/conf/adapter.conf",
              "adapter of planning configure file name");
DEFINE_string(planning_adapter_conf_debug_file, "../components/planning/conf/adapter_debug.conf",
              "adapter of planning configure file name");
DEFINE_string(planner_conf_file, "../components/planning/conf/planning_config.pb.txt",
              "planning configure file name" );
DEFINE_int32(planning_cycle, 100, "the cycle time of planning");

DEFINE_double(top_speed, 3.0, "top speed");

