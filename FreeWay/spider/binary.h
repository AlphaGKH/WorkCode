#ifndef SPIDER_BINARY_H_
#define SPIDER_BINARY_H_

#include <string>

namespace spider {
class Binary {
public:
    static std::string GetName() { return GetNameRef(); }
    static void SetName(const std::string& name) { GetNameRef() = name; }
    static std::string& GetNameRef() {
        static std::string binary_name;
        return binary_name;
    }
};
}  // namespace spider

#endif  //
