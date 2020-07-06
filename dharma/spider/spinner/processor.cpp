#include "spider/spinner/processor.h"

#include "spider/base/macros.h"

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





}
