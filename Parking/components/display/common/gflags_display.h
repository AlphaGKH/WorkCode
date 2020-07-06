#ifndef DISPLAY_COMMON_GFLAGS_DISPLAY_H_
#define DISPLAY_COMMON_GFLAGS_DISPLAY_H_

#include "gflags/gflags.h"

DECLARE_string(display_adapter_conf_file);
DECLARE_string(display_adapter_conf_debug_file);
DECLARE_string(display_conf_file);
DECLARE_int32(display_cycle);

// for debug
DECLARE_bool(enable_global_view);
DECLARE_bool(enable_local_view);





#endif
