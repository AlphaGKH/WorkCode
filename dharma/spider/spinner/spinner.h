#ifndef SPIDER_SPINNER_SPINNER_H_
#define SPIDER_SPINNER_SPINNER_H_

#include <memory>
#include <unordered_map>

#include "spider/common/macros.h"

#include "spider/spinner/processor.h"

namespace spider {

class Spinner {
public:
    ~Spinner(){};

public:
    bool CreateTask(const std::string& task_name, TaskFunc && func);

    bool RemoveTask(const std::string& task_name);

    void Shutdown();

private:
    std::unordered_map<std::string, std::shared_ptr<Processor> > processors_;
    std::atomic_bool stop_;

    DECLARE_SINGLETON(Spinner)
};

}

#endif
