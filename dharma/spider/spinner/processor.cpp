#include "spider/spinner/processor.h"

#include "spider/base/macros.h"
#include "spider/common/log.h"

#include "spider/time/time.h"

namespace spider {

Processor::Processor() {
    running_.store(true);
}

Processor::~Processor() {
    Stop();
}

void Processor::Run() {
    while (spider_likely(running_.load())) {
        if(spider_likely(task_ != nullptr)) {
            task_();
        }
        else {
            std::unique_lock<std::mutex> lk(mtx_task_);
            cv_task_.wait_for(lk, std::chrono::milliseconds(10));
        }
    }
}

void Processor::TimerRun(){
    while (spider_likely(running_.load())) {
        std::this_thread::yield();

        if(last_cost_time_ > cycle_time_ * 1000000) {
            AWARN << "It took Too Long Time in last cycle!";
        }
        else {
            std::this_thread::sleep_for(
                        std::chrono::duration<uint64_t, std::milli>(cycle_time_ - last_cost_time_ / 1000000));
        }

        if(spider_likely(task_ != nullptr)) {
            uint64_t start_timestamp = Time::Now().ToNanosecond();
            task_();
            last_cost_time_ = Time::Now().ToNanosecond() - start_timestamp;
        }
        else {
            std::unique_lock<std::mutex> lk(mtx_task_);
            cv_task_.wait_for(lk, std::chrono::milliseconds(10));
        }
    }
}

void Processor::Stop() {
    if (!running_.exchange(false)) {
        return;
    }
    task_ = nullptr;

    cv_task_.notify_one();
    if(thread_ && thread_->joinable()){
        thread_->join();
    }
}

void Processor::BindTask(TaskFunc &task) {
    if(task == nullptr){
        return;
    }
    task_ = task;
    std::call_once(thread_flag_,
                   [this](){
        thread_ = std::make_shared<std::thread>(&Processor::Run, this);
    });
}

void Processor::BindTimerTask(TaskFunc &task, const uint64_t &cycle_time){
    if(task == nullptr){
        return;
    }
    task_ = task;
    cycle_time_ = cycle_time;
    std::call_once(thread_flag_,
                   [this](){
        thread_ = std::make_shared<std::thread>(&Processor::TimerRun, this);
    });
}





}
