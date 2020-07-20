/**
  * @projectName : LeapAuto
  * @brief       :
  * @author      : guokonghui
  * @date        : 2019-10-11
  */

#pragma once

#include <vector>
#include <stdint.h>
#include <cmath>
#include <string>

namespace dharma {
namespace common {
namespace util {

template <typename ProtoA, typename ProtoB>
bool IsProtoEqual(const ProtoA& a, const ProtoB& b) {
    return a.GetTypeName() == b.GetTypeName() &&
            a.SerializeAsString() == b.SerializeAsString();
    // Test shows that the above method is 5 times faster than the
    // API: google::protobuf::util::MessageDifferencer::Equals(a, b);
}

template <typename U, typename V>
double DistanceXY(const U& u, const V& v) {
    return std::hypot(u.x() - v.x(), u.y() - v.y());
}


template <typename T>
void uniform_slice(const T start, const T end, uint32_t num,
                   std::vector<T>* sliced) {
    if (!sliced || num == 0) {
        return;
    }
    const T delta = (end - start) / num;
    sliced->resize(num + 1);
    T s = start;
    for (uint32_t i = 0; i < num; ++i, s += delta) {
        sliced->at(i) = s;
    }
    sliced->at(num) = end;
}
}
}

}


