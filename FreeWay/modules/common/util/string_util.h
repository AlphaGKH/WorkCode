#pragma once

#include <string>

#include "google/protobuf/stubs/stringprintf.h"

/**
 * @namespace dharma::common::util
 * @brief dharma::common::util
 */
namespace dharma {
namespace common {
namespace util {

// TODO(xiaoxq): Migrate to absl::StrFormat after absl upgraded.
using google::protobuf::StringPrintf;

struct DebugStringFormatter {
    template <class T>
    void operator()(std::string* out, const T& t) const {
        out->append(t.DebugString());
    }
};

//std::string EncodeBase64(std::string_view in);

}  // namespace util
}  // namespace common
}  // namespace dharma
