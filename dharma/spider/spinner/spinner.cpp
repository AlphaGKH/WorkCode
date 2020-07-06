#include "spider/spinner/spinner.h"

namespace spider {

Spinner::Spinner() : stop_(false){}

bool Spinner::CreateTask(const std::string& task_name, TaskFunc &&func) {

    auto proc = std::make_shared<Processor>();
    proc->BindTask(func);

    if(processors_.find(task_name) != processors_.end()){
        return false;
    }
    processors_[task_name] = proc;
    return true;
}

bool Spinner::CreateTimerTask(const std::string &task_name, TaskFunc &&func, const uint64_t& cycle_time) {
    auto proc = std::make_shared<Processor>();
    proc->BindTimerTask(func, cycle_time);

    if(processors_.find(task_name) != processors_.end()){
        return false;
    }
    processors_[task_name] = proc;
    return true;
}

bool Spinner::RemoveTask(const std::string &task_name) {
    if(spider_unlikely(stop_)){
        return true;
    }

    if(processors_.find(task_name) != processors_.end()){
        processors_[task_name]->Stop();
        processors_.erase(task_name);
        return true;
    }
    else {
        return false;
    }
}

void Spinner::Shutdown(){
    if (spider_unlikely(stop_.exchange(true))) {
        return;
    }

    for(auto p : processors_){
        p.second->Stop();
    }

    processors_.clear();
}

}
