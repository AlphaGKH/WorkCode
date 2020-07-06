#include "control/common/gflags_control.h"

DEFINE_string(control_adapter_conf_file, "../components/control/conf/adapter.conf",
              "adapter of control configure file name");

DEFINE_string(control_conf_file, "../components/control/conf/control_config.pb.txt", "control configure file");

DEFINE_int32(control_cycle, 10, "the cycle time of control");

