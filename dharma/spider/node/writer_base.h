#ifndef SPIDER_NODE_WRITER_BASE_H_
#define SPIDER_NODE_WRITER_BASE_H_

#include <atomic>
#include <string>
#include <memory>

#include "lcm/lcm-cpp.hpp"

namespace spider {

class WriterBase {
public:
    explicit WriterBase(const std::string &channel_name, const uint32_t& depth)
        : channel_name_(channel_name),
          depth_(depth),
          init_(false),
          lcm_ptr_(nullptr){}

    virtual ~WriterBase() {}

public:
    virtual bool Init() = 0;

    virtual void Shutdown() = 0;

    bool IsInit() const { return init_.load(); }

    const std::string& GetChannelName() const {
        return channel_name_;
    }

protected:
    std::string channel_name_;
    uint32_t depth_;
    std::atomic_bool init_;
    std::unique_ptr<lcm::LCM> lcm_ptr_;
};

} // namespace spider

#endif
