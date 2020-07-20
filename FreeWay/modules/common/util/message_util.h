#pragma once

#include "google/protobuf/message.h"

#include "spider/time/time.h"

namespace dharma {

namespace common {

namespace util {

template <typename T, typename std::enable_if<
              std::is_base_of<google::protobuf::Message, T>::value,
              int>::type = 0>
static void FillHeader(const std::string& module_name, T* msg) {
    static std::atomic<uint64_t> sequence_num = {0};
    auto* header = msg->mutable_header();
    double timestamp = spider::Time::Now().ToSecond();
    header->set_module_name(module_name);
    header->set_timestamp_sec(timestamp);
    header->set_sequence_num(
                static_cast<unsigned int>(sequence_num.fetch_add(1)));
}

}

}

}
