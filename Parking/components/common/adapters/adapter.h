#ifndef COMMON_ADAPTERS_ADAPTER_H_
#define COMMON_ADAPTERS_ADAPTER_H_

#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <list>

#include <iostream>

#include "lcm/lcm-cpp.hpp"

#include "common/mlog/mlog.h"
#include "common/time/time.h"


namespace common {

namespace adapter {


template <typename D>
class Adapter
{
public:
    typedef D DataType;
    typedef std::shared_ptr<D const> DataPtr;
    typedef typename std::list<DataPtr>::const_iterator Iterator;

    Adapter(const std::string& channel_name, const size_t& message_num)
        : channel_name_("CHANNEL_" + channel_name),
          message_num_(message_num){}

    ~Adapter(){
        if(sub_thread_ && sub_thread_->joinable()){
            sub_thread_->join();
        }
    }

/// for subscribe
public:
    bool HasReceived() const  {
        std::lock_guard<std::mutex> lock(mutex_);
        return !data_queue_.empty();
    }

    void Observe() {
        std::lock_guard<std::mutex> lock(mutex_);
        observed_queue_ = data_queue_;
    }

    bool Empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return observed_queue_.empty();
    }

    const D& GetLatestObserved() const {
        std::lock_guard<std::mutex> lock(mutex_);
        DCHECK(!observed_queue_.empty())
                << "The view of data queue is empty. No data is received yet or you forgot to call Observe()"
                << ":" << channel_name_;
        return *observed_queue_.front();
    }

    const D& GetOldestObserved() const {
        std::lock_guard<std::mutex> lock(mutex_);
        DCHECK(!observed_queue_.empty())
                << "The view of data queue is empty. No data is received yet or you "
                   "forgot to call Observe().";
        return *observed_queue_.back();
    }

    double GetDelaySec() const  {
        if (last_receive_time_ == 0) {
            return -1;
        } else {
            return common::time::Clock::NowInSeconds() - last_receive_time_;
        }
    }

    void ClearData() {
        std::lock_guard<std::mutex> lock(mutex_);
        data_queue_.clear();
        observed_queue_.clear();
    }

private:
    void EnqueueData(DataPtr data){
        if (message_num_ == 0) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (data_queue_.size() + 1 > message_num_) {
            data_queue_.pop_back();
        }
        data_queue_.push_front(data);
//        std::cout << "size: " << data_queue_.size() << std::endl;
    }

    void OnReceive(const lcm::ReceiveBuffer *rbuf, const std::string &chan,const D *msg){
        last_receive_time_ = common::time::Clock::NowInSeconds();
        //        std::cout << "receive_thread_id: " << std::this_thread::get_id()
        //                  << "dalay_time: " << common::time::Clock::NowInSeconds() - msg->timestamp
        //                  << std::endl;
        EnqueueData(std::make_shared<D const>(*msg));
    }

    void SubThreadRun(){
        while (!sub_thread_stop_) {
            std::this_thread::yield();
            int result = lcm_handle_->handleTimeout(100);
            if (0 < result) {
                update_counter_ = 10;
                continue;
            }
            else if (0 == result){
                update_counter_ --;
//                AWARN << "Lcm Subscribe for " << channel_name_ << " TimeOut";
            }else {
                update_counter_ = -1;
                AERROR << "Lcm Subscribe for " << channel_name_ << " Error occured!";
            }

            if(update_counter_ < 0){
                ClearData();
                AERROR << "Lcm Subscribe for " << channel_name_ << " has Not receive message for 10 cycles!";
            }

        }
    }

    void Subscribe(){
        lcm_handle_->subscribe(channel_name_,&Adapter::OnReceive,this);
        sub_thread_ = std::make_unique<std::thread>(&Adapter::SubThreadRun, this);
    }

private:
    size_t message_num_;
    mutable std::mutex mutex_;
    std::list<DataPtr> data_queue_;
    std::list<DataPtr> observed_queue_;
    double last_receive_time_ = 0;
    std::unique_ptr<std::thread> sub_thread_;
    bool sub_thread_stop_ = false;


/// for publish
public:
    const D GetLatestPublished() const { return *latest_published_data_; }

private:
    void Publish(const D& data){
        lcm_handle_->publish(channel_name_,&data);
    }

    void SetLatestPublished(const D& data) {
        latest_published_data_.reset(new D(data));
    }

private:
    std::unique_ptr<D> latest_published_data_;

/// common
public:
    bool EnableLcm(){
        lcm_handle_ = std::make_unique<lcm::LCM>();
        if(!lcm_handle_->good()){
            AERROR << "Lcm for " << channel_name_ << " is NOT good!";
            return false;
        }

        return true;
    }

private:
    std::string channel_name_;

    std::unique_ptr<lcm::LCM> lcm_handle_;

    int update_counter_ = 10;

    friend class AdapterManager;

};

}

}



#endif
