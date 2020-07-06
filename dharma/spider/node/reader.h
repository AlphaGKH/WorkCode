#ifndef SPIDER_NODE_READER_H_
#define SPIDER_NODE_READER_H_

#include <list>
#include <functional>
#include <iostream>

#include "spider/node/reader_base.h"
#include "spider/base/macros.h"
#include "spider/time/time.h"
#include "spider/spinner/spinner.h"

namespace spider {

namespace {

DEFINE_TYPE_TRAIT(HasParseFromArray, ParseFromArray)

template <typename T>
typename std::enable_if<HasParseFromArray<T>::value, bool>::type ParseFromArray(
        const void* data, int size, T* message) {
    return message->ParseFromArray(data, size);
}

template <typename T>
typename std::enable_if<!HasParseFromArray<T>::value, bool>::type ParseFromArray(
        const void* data, int size, T* message) {
    return message->decode(data, 0, size);
}

}

template <typename T>
using CallbackFunc = std::function<void(const std::shared_ptr<T>&)>;

template <typename MessageT>
class Reader : public ReaderBase {
public:
    explicit Reader(const std::string &channel_name,
                    const CallbackFunc<MessageT>& reader_func,
                    const uint32_t& depth,
                    const uint32_t& time_out)
        : ReaderBase(channel_name, depth, time_out),
          reader_func_(reader_func){}
    virtual ~Reader() {
        Shutdown();
    };

public:
    bool Init() override;

    void Shutdown() override;

    void Observe() override;

    bool HasReceived() const override;

    bool Empty() const override;

    double GetDelaySec() const override;

    /**
     * @brief Get the latest message we `Observe`
     *
     * @return std::shared_ptr<MessageT> the latest message
     */
    virtual std::shared_ptr<MessageT> GetLatestObserved() const;

    /**
     * @brief Get the oldest message we `Observe`
     *
     * @return std::shared_ptr<MessageT> the oldest message
     */
    virtual std::shared_ptr<MessageT> GetOldestObserved() const;

    void ClearData() override;

private:
    virtual void Enqueue(const std::shared_ptr<MessageT>& msg_ptr);

    virtual void OnReceive(const lcm::ReceiveBuffer* rbuf, const std::string& channel);

private:
    std::list<std::shared_ptr<MessageT>> received_queue_;
    std::list<std::shared_ptr<MessageT>> observed_queue_;

    CallbackFunc<MessageT> reader_func_;

    int update_counter_ = 10;
};

template<typename MessageT>
bool Reader<MessageT>::Init(){
    if(init_.exchange(true)){
        return true;
    }

    lcm_ptr_ = std::make_unique<lcm::LCM>();

    if((lcm_ptr_ == nullptr) || (!lcm_ptr_->good())){
        init_.store(false);
        return false;
    }

    lcm_ptr_->subscribe(channel_name_, &Reader::OnReceive, this);

    Spinner::Instance()->CreateTask(channel_name_,
                                    [this]() {
        int res = lcm_ptr_->handleTimeout(time_out_);
        if (res > 0) {
            update_counter_ = 10;
            return;
        }
        else if (res == 0 ){
            update_counter_--;
            AWARN << "Reader for " << channel_name_ << " TimeOut";
        }
        else {
            update_counter_ = -1;
            AERROR << "Reader for " << channel_name_ << " Error occured!";
        }

        if(update_counter_ < 0){
            ClearData();
        }
        return;
    });

    return true;
}

template<typename MessageT>
void Reader<MessageT>::Shutdown() {
    if(!init_.exchange(false)){
        return;
    }

    lcm_ptr_ = nullptr;

    Spinner::Instance()->RemoveTask(channel_name_);
}

template<typename MessageT>
void Reader<MessageT>::Enqueue(const std::shared_ptr<MessageT>& msg) {
    second_to_lastest_recv_time_sec_ = latest_recv_time_sec_;
    latest_recv_time_sec_ = Time::Now().ToSecond();

    if(depth_ == 0){
        return;
    }

    std::lock_guard<std::mutex> lock(msg_mutex_);

    received_queue_.push_front(msg);

    while (received_queue_.size() > depth_) {
        received_queue_.pop_back();
    }
}

template <typename MessageT>
void Reader<MessageT>::Observe() {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    observed_queue_ = received_queue_;

    return;
}

template <typename MessageT>
bool Reader<MessageT>::HasReceived() const {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    return !received_queue_.empty();
}

template <typename MessageT>
bool Reader<MessageT>::Empty() const {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    return received_queue_.empty();
}

template <typename MessageT>
double Reader<MessageT>::GetDelaySec() const {
    if (latest_recv_time_sec_ < 0) {
        return -1.0;
    }
    if (second_to_lastest_recv_time_sec_ < 0) {
        return Time::Now().ToSecond() - latest_recv_time_sec_;
    }
    return std::max((Time::Now().ToSecond() - latest_recv_time_sec_),
                    (latest_recv_time_sec_ - second_to_lastest_recv_time_sec_));
}

template <typename MessageT>
std::shared_ptr<MessageT> Reader<MessageT>::GetLatestObserved() const {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    if (observed_queue_.empty()) {
        return nullptr;
    }

    return observed_queue_.front();
}

template <typename MessageT>
std::shared_ptr<MessageT> Reader<MessageT>::GetOldestObserved() const {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    if (observed_queue_.empty()) {
        return nullptr;
    }
    return observed_queue_.back();
}

template <typename MessageT>
void Reader<MessageT>::ClearData() {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    received_queue_.clear();
    observed_queue_.clear();
}

template<typename MessageT>
void Reader<MessageT>::OnReceive(const lcm::ReceiveBuffer* rbuf, const std::string& channel){
    auto msg = std::make_shared<MessageT>();

    RETURN_IF(!ParseFromArray(rbuf->data, static_cast<int>(rbuf->data_size), msg.get()));
    Enqueue(msg);
    if(reader_func_ != nullptr){
        reader_func_(msg);
    }
}

}

#endif
