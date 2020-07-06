#ifndef SPIDER_SPINNER_PROCESSOR_H_
#define SPIDER_SPINNER_PROCESSOR_H_

#include <functional>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>

using TaskFunc = std::function<void()>;


namespace spider {

class Processor {
public:
    Processor();
    virtual ~Processor();

public:
    void Run();
    void Stop();
    void BindTask(TaskFunc& task);

public:
    void TimerRun();

    void BindTimerTask(TaskFunc& task, const uint64_t& cycle_time);

private:
    TaskFunc task_ = nullptr;
    uint64_t cycle_time_ = 0;

    uint64_t last_cost_time_ = 0;

    std::atomic_bool running_{false};
    std::once_flag thread_flag_;
    std::shared_ptr<std::thread> thread_;

    std::condition_variable cv_task_;
    std::mutex mtx_task_;
};

}

#endif
