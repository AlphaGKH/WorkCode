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

private:
    TaskFunc task_ = nullptr;
    std::atomic_bool running_{false};
    std::once_flag thread_flag_;
    std::shared_ptr<std::thread> thread_;

    std::condition_variable cv_task_;
    std::mutex mtx_task_;
};

}

#endif
