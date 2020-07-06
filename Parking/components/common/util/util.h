/**
  * @projectName : LeapAuto
  * @brief       :
  * @author      : guokonghui
  * @date        : 2019-10-11
  */

#ifndef COMMON_UTILS_UTIL_H_
#define COMMON_UTILS_UTIL_H_

#include <vector>
#include <stdint.h>
#include <cmath>
#include <string>

namespace common {

namespace util {

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


struct DebugStringFormatter {
    template <class T>
    void operator()(std::string* out, const T& t) const {
        out->append(t.DebugString());
    }
};

}



}

#endif


