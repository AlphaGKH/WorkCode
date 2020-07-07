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
private:
    std::shared_ptr<Reader<M0> > reader_ptr0_ = nullptr;
    std::shared_ptr<Reader<M1> > reader_ptr1_ = nullptr;
    std::shared_ptr<Reader<M2> > reader_ptr2_ = nullptr;
    std::shared_ptr<Reader<M3> > reader_ptr3_ = nullptr;
};

// one message
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

    reader_ptr_ = node_->CreateReader<M0>(config.readers(0).channel(),
                                          config.readers(0).pending_queue_size(),
                                          config.readers(0).time_out());
    if (reader_ptr_ == nullptr) {
        AERROR << "Component create reader failed.";
        return false;
    }

    readers_.emplace_back(reader_ptr_);

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

    if(reader_ptr_->Empty()){
        AWARN << "Message of " << reader_ptr_->GetChannelName() << " is not ready!" << std::endl;
        return false;
    }

    auto msg = reader_ptr_->GetLatestObserved();

    return Proc(msg);
}

// two message
template <typename M0, typename M1>
class Component<M0, M1, NullType, NullType> : public ComponentBase {
public:
    Component() {}
    ~Component() override {}

public:
    bool Initialize(const ComponentConfig& config) override;
    bool Process();

private:
    virtual bool Proc(const std::shared_ptr<M0>& msg0,
                      const std::shared_ptr<M1>& msg1) = 0;

    std::shared_ptr<Reader<M0> > reader_ptr0_ = nullptr;
    std::shared_ptr<Reader<M1> > reader_ptr1_ = nullptr;
};

template <typename M0, typename M1>
bool Component<M0, M1, NullType, NullType>::Initialize(const ComponentConfig& config) {
    node_.reset(new Node(config.name()));

    if(config.readers_size() < 1){
        AERROR << "Invalid config file: too few readers.";
        return false;
    }

    if (!Init()) {
        AERROR << "Component Init() failed.";
        return false;
    }

    reader_ptr0_ = node_->CreateReader<M0>(config.readers(0).channel(),
                                           config.readers(0).pending_queue_size(),
                                           config.readers(0).time_out());
    reader_ptr1_ = node_->CreateReader<M1>(config.readers(1).channel(),
                                           config.readers(1).pending_queue_size(),
                                           config.readers(1).time_out());
    if (reader_ptr0_ == nullptr || reader_ptr1_ == nullptr) {
        AERROR << "Component create reader failed.";
        return false;
    }

    readers_.emplace_back(reader_ptr0_);
    readers_.emplace_back(reader_ptr1_);

    Spinner::Instance()->CreateTask(node_->Name(),[this](){
        Process();
    });

    return true;
}

template <typename M0, typename M1>
bool Component<M0, M1, NullType, NullType>::Process() {
    if (is_shutdown_.load()) {
        return true;
    }
    node_->Observe();

    if(reader_ptr0_->Empty()){
        AWARN << "Message of " << reader_ptr0_->GetChannelName() << " is not ready!" << std::endl;
        return false;
    }

    if(reader_ptr1_->Empty()){
        AWARN << "Message of " << reader_ptr1_->GetChannelName() << " is not ready!" << std::endl;
        return false;
    }

    auto msg0 = reader_ptr0_->GetLatestObserved();
    auto msg1 = reader_ptr1_->GetLatestObserved();

    return Proc(msg0, msg1);
}

// three message
template <typename M0, typename M1, typename M2>
class Component<M0, M1, M2, NullType> : public ComponentBase {
public:
    Component() {}
    ~Component() override {}

public:
    bool Initialize(const ComponentConfig& config) override;
    bool Process();

private:
    virtual bool Proc(const std::shared_ptr<M0>& msg0,
                      const std::shared_ptr<M1>& msg1,
                      const std::shared_ptr<M2>& msg2) = 0;

    std::shared_ptr<Reader<M0> > reader_ptr0_ = nullptr;
    std::shared_ptr<Reader<M1> > reader_ptr1_ = nullptr;
    std::shared_ptr<Reader<M2> > reader_ptr2_ = nullptr;
};

template <typename M0, typename M1, typename M2>
bool Component<M0, M1, M2, NullType>::Initialize(const ComponentConfig& config) {
    node_.reset(new Node(config.name()));

    if(config.readers_size() < 1){
        AERROR << "Invalid config file: too few readers.";
        return false;
    }

    if (!Init()) {
        AERROR << "Component Init() failed.";
        return false;
    }

    reader_ptr0_ = node_->CreateReader<M0>(config.readers(0).channel(),
                                           config.readers(0).pending_queue_size(),
                                           config.readers(0).time_out());
    reader_ptr1_ = node_->CreateReader<M1>(config.readers(1).channel(),
                                           config.readers(1).pending_queue_size(),
                                           config.readers(1).time_out());
    reader_ptr2_ = node_->CreateReader<M2>(config.readers(2).channel(),
                                           config.readers(2).pending_queue_size(),
                                           config.readers(2).time_out());
    if (reader_ptr0_ == nullptr || reader_ptr1_ == nullptr || reader_ptr2_ == nullptr) {
        AERROR << "Component create reader failed.";
        return false;
    }

    readers_.emplace_back(reader_ptr0_);
    readers_.emplace_back(reader_ptr1_);
    readers_.emplace_back(reader_ptr2_);

    Spinner::Instance()->CreateTask(node_->Name(),[this](){
        Process();
    });

    return true;
}

template <typename M0, typename M1, typename M2>
bool Component<M0, M1, M2, NullType>::Process() {
    if (is_shutdown_.load()) {
        return true;
    }
    node_->Observe();

    if(reader_ptr0_->Empty()){
        AWARN << "Message of " << reader_ptr0_->GetChannelName() << " is not ready!" << std::endl;
        return false;
    }

    if(reader_ptr1_->Empty()){
        AWARN << "Message of " << reader_ptr1_->GetChannelName() << " is not ready!" << std::endl;
        return false;
    }

    if(reader_ptr2_->Empty()){
        AWARN << "Message of " << reader_ptr2_->GetChannelName() << " is not ready!" << std::endl;
        return false;
    }

    auto msg0 = reader_ptr0_->GetLatestObserved();
    auto msg1 = reader_ptr1_->GetLatestObserved();
    auto msg2 = reader_ptr2_->GetLatestObserved();

    return Proc(msg0, msg1, msg2);
}

// four message
template <typename M0, typename M1, typename M2, typename M3>
bool Component<M0, M1, M2, M3>::Initialize(const ComponentConfig& config) {
    node_.reset(new Node(config.name()));

    if(config.readers_size() < 1){
        AERROR << "Invalid config file: too few readers.";
        return false;
    }

    if (!Init()) {
        AERROR << "Component Init() failed.";
        return false;
    }

    reader_ptr0_ = node_->CreateReader<M0>(config.readers(0).channel(),
                                           config.readers(0).pending_queue_size(),
                                           config.readers(0).time_out());
    reader_ptr1_ = node_->CreateReader<M1>(config.readers(1).channel(),
                                           config.readers(1).pending_queue_size(),
                                           config.readers(1).time_out());
    reader_ptr2_ = node_->CreateReader<M2>(config.readers(2).channel(),
                                           config.readers(2).pending_queue_size(),
                                           config.readers(2).time_out());
    reader_ptr3_ = node_->CreateReader<M3>(config.readers(3).channel(),
                                           config.readers(3).pending_queue_size(),
                                           config.readers(3).time_out());
    if (reader_ptr0_ == nullptr || reader_ptr1_ == nullptr || reader_ptr2_ == nullptr || reader_ptr3_ == nullptr) {
        AERROR << "Component create reader failed.";
        return false;
    }

    readers_.emplace_back(reader_ptr0_);
    readers_.emplace_back(reader_ptr1_);
    readers_.emplace_back(reader_ptr2_);
    readers_.emplace_back(reader_ptr3_);

    Spinner::Instance()->CreateTask(node_->Name(),[this](){
        Process();
    });

    return true;
}

template <typename M0, typename M1, typename M2, typename M3>
bool Component<M0, M1, M2, M3>::Process() {
    if (is_shutdown_.load()) {
        return true;
    }
    node_->Observe();

    if(reader_ptr0_->Empty()){
        AWARN << "Message of " << reader_ptr0_->GetChannelName() << " is not ready!" << std::endl;
        return false;
    }

    if(reader_ptr1_->Empty()){
        AWARN << "Message of " << reader_ptr1_->GetChannelName() << " is not ready!" << std::endl;
        return false;
    }

    if(reader_ptr2_->Empty()){
        AWARN << "Message of " << reader_ptr2_->GetChannelName() << " is not ready!" << std::endl;
        return false;
    }

    if(reader_ptr3_->Empty()){
        AWARN << "Message of " << reader_ptr3_->GetChannelName() << " is not ready!" << std::endl;
        return false;
    }

    auto msg0 = reader_ptr0_->GetLatestObserved();
    auto msg1 = reader_ptr1_->GetLatestObserved();
    auto msg2 = reader_ptr2_->GetLatestObserved();
    auto msg3 = reader_ptr2_->GetLatestObserved();

    return Proc(msg0, msg1, msg2, msg3);
}

}

#define MODULEMAIN(component, configure_file)                   \
int main(){                                                     \
    spider::ComponentConfig config;                             \
    spider::common::GetProtoFromFile(configure_file, &config);  \
    spider::Init(config.name().c_str());                        \
    std::shared_ptr<spider::ComponentBase> base = nullptr;      \
    base.reset(new component());                                \
    if(base == nullptr || !base->Initialize(config)){           \
        return 0;                                               \
    }                                                           \
    spider::WaitForShutdown();                                  \
}

#endif
