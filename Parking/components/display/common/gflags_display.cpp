#include "display/common/gflags_display.h"

DEFINE_string(display_adapter_conf_file, "../components/display/conf/adapter.conf",
              "adapter of display configure file name");

DEFINE_string(display_adapter_conf_debug_file, "../components/display/conf/adapter_debug.conf",
              "adapter of display configure file name in debug mode");

DEFINE_string(display_conf_file, "../components/display/conf/display_config.pb.txt",
              "display configure file name");

DEFINE_int32(display_cycle, 100, "the cycle time of display");

// for debug
DEFINE_bool(enable_global_view, true, "whether enable global view");
DEFINE_bool(enable_local_view, false, "whether enable local view");

