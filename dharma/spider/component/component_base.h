#ifndef SPIDER_COMPONENT_COMPONENT_BASE_H_
#define SPIDER_COMPONENT_COMPONENT_BASE_H_

#include "spider/common/file.h"
#include "spider/node/node.h"

#include "spider/component_conf.pb.h"

namespace spider {

using spider::proto::ComponentConfig;

class ComponentBase {
public:
    ComponentBase() = default;
    virtual ~ComponentBase() {}

public:
    virtual bool Initialize(const ComponentConfig& config) { return false; }

    virtual void Shutdown() {
        if(is_shutdown_.exchange(true)){
            return;
        }
        Clear();
        for(auto& reader : readers_){
            reader->Shutdown();
        }

        Spinner::Instance()->RemoveTask(node_->Name());
    }

protected:
    virtual bool Init() = 0;
    virtual void Clear() { return; }

protected:
    std::atomic_bool is_shutdown_ = {false};
    std::unique_ptr<Node> node_ = nullptr;

    std::vector<std::shared_ptr<ReaderBase> > readers_;

};



}


#endif
