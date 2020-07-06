#ifndef SPIDER_NODE_READER_BASE_H_
#define SPIDER_NODE_READER_BASE_H_

#include <string>
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "lcm/lcm-cpp.hpp"

#include "spider/common/log.h"

namespace spider {

class  ReaderBase {
public:
    explicit ReaderBase(const std::string &channel_name,
                        const uint32_t& depth, const uint32_t& time_out)
        : channel_name_(channel_name),
          depth_(depth),
          time_out_(time_out),
          init_(false),
          lcm_ptr_(nullptr){}

    virtual ~ReaderBase(){}
public:
    virtual bool Init() = 0;

    virtual void Shutdown() = 0;

    bool IsInit() const { return init_.load(); }

    const std::string& GetChannelName() const {
        return channel_name_;
    }

    uint32_t GetHistoryDepth() const {
        return depth_;
    }

    /**
     * @brief Get stored data
     */
    virtual void Observe() = 0;

    /**
     * @brief Query whether we have received data since last clear
     *
     * @return true if the reader has received data
     * @return false if the reader has not received data
     */
    virtual bool HasReceived() const = 0;

    /**
     * @brief Query whether the Reader has data to be handled
     *
     * @return true if data container is empty
     * @return false if data container has data
     */
    virtual bool Empty() const = 0;

    /**
     * @brief Get time interval of since last receive message
     *
     * @return double seconds delay
     */
    virtual double GetDelaySec() const = 0;

    /**
     * @brief Get the value of pending queue size
     *
     * @return uint32_t result value
     */
    virtual uint32_t PendingQueueSize() const {
        return depth_;
    };

    /**
     * @brief Clear local data
     */
    virtual void ClearData() = 0;

protected:
    std::string channel_name_;
    uint32_t depth_;
    std::atomic_bool init_;
    std::unique_ptr<lcm::LCM> lcm_ptr_;

    double latest_recv_time_sec_ = -1.0;
    double second_to_lastest_recv_time_sec_ = -1.0;
    int time_out_ = 500; //ms

    mutable std::mutex msg_mutex_;
};

}

#endif
