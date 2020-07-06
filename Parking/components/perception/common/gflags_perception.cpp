#include "perception/common/gflags_perception.h"

DEFINE_string(perception_adapter_conf_file, "../components/perception/conf/adapter.conf",
              "adapter of perception configure file name");

DEFINE_string(perception_conf_file, "../components/perception/conf/perception_config.pb.txt",
              "ogm configure file name");

DEFINE_int32(perception_cycle, 100, "perception cycle time");

