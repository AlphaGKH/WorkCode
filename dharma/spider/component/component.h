#ifndef SPIDER_COMPONENT_COMPONENT_H_
#define SPIDER_COMPONENT_COMPONENT_H_

#include "spider/component/component_base.h"

namespace spider {

class NullType {};

template <typename M0 = NullType, typename M1 = NullType,
          typename M2 = NullType, typename M3 = NullType>
class Component : public ComponentBase {
public:
    Component() = default;
    ~Component() override {}

    /**
     * @brief init the component by protobuf object.
     *
     * @param config which is defined in 'cyber/proto/component_conf.proto'
     *
     * @return returns true if successful, otherwise returns false
     */
    bool Initialize(const ComponentConfig& config) override;
    bool Process();

private:
    /**
     * @brief The process logical of yours.
     *
     * @param msg0 the first channel message.
     * @param msg1 the second channel message.
     * @param msg2 the third channel message.
     * @param msg3 the fourth channel message.
     *
     * @return returns true if successful, otherwise returns false
     */
    virtual bool Proc(const std::shared_ptr<M0>& msg0,
                      const std::shared_ptr<M1>& msg1,
                      const std::shared_ptr<M2>& msg2,
                      const std::shared_ptr<M3>& msg3) = 0;
};

template <typename M0>
class Component<M0, NullType, NullType, NullType> : public ComponentBase {
public:
    Component() {}
    ~Component() override {}

public:
    bool Initialize(const ComponentConfig& config) override;
    bool Process();

private:
    virtual bool Proc(const std::shared_ptr<M0>& msg) = 0;

    std::shared_ptr<Reader<M0> > reader_ptr_ = nullptr;
};

template <typename M0>
bool Component<M0, NullType, NullType, NullType>::Initialize(const ComponentConfig& config) {
    node_.reset(new Node(config.name()));

    if(config.readers_size() < 1){
        AERROR << "Invalid config file: too few readers.";
        return false;
    }

    if (!Init()) {
        AERROR << "Component Init() failed.";
        return false;
    }

    std::shared_ptr<Reader<M0>> reader = nullptr;

    reader = node_->CreateReader<M0>(config.readers(0).channel(),
                                     config.readers(0).pending_queue_size(),
                                     config.readers(0).time_out());
    if (reader == nullptr) {
        AERROR << "Component create reader failed.";
        return false;
    }

    readers_.emplace_back(reader);

    reader_ptr_.reset(reader.get());

    Spinner::Instance()->CreateTask(node_->Name(),[this](){
        Process();
    });

    return true;
}

template <typename M0>
bool Component<M0, NullType, NullType, NullType>::Process() {
    if (is_shutdown_.load()) {
        return true;
    }
    node_->Observe();

    std::shared_ptr<M0> msg = nullptr;
    if(reader_ptr_ != nullptr){
        msg = reader_ptr_->GetLatestObserved();

        if(msg){
            return Proc(msg);
        }

    }

//    std::shared_ptr<M0> msg = reader_ptr_->GetLatestObserved();

//    auto r = readers_[0];


//    Reader<M0>* reader = dynamic_cast<Reader<M0>* >(r.get());

//    std::shared_ptr<M0> msg = reader->GetLatestObserved();

//    auto msg = reader_->GetLatestObserved();

//    auto msg = reader_ptr->GetLatestObserved();  << msg->content()

//    std::cout << "Process Running!" << std::endl;

//    std::cout << msg->content() << std::endl;


//    return Proc(msg);

    return true;
}

}

#endif
