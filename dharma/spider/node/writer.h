#ifndef SPIDER_NODE_WRITER_H_
#define SPIDER_NODE_WRITER_H_

#include "spider/node/writer_base.h"
#include "spider/common/log.h"
#include "spider/base/macros.h"

namespace spider {

namespace {

DEFINE_TYPE_TRAIT(HasSerializeToArray, SerializeToArray)
DEFINE_TYPE_TRAIT(HasByteSize, ByteSize)

template <typename T>
typename std::enable_if<HasSerializeToArray<T>::value, bool>::type
SerializeToArray(const T& message, void* data, int size) {
    return message.SerializeToArray(data, size);
}

template <typename T>
typename std::enable_if<!HasSerializeToArray<T>::value, bool>::type
SerializeToArray(const T& message, void* data, int size) {
    return message.encode(data, 0, size);
}

template <typename T>
typename std::enable_if<HasByteSize<T>::value, int>::type ByteSizeLong(
        const T& message) {
    return message.ByteSizeLong();
}

template <typename T>
typename std::enable_if<!HasByteSize<T>::value, int>::type ByteSizeLong(
        const T& message) {
    return message.getEncodedSize();
}

}

template <typename MessageT>
class Writer : public WriterBase {
public:
    explicit Writer(const std::string &channel_name, const uint32_t& depth)
        : WriterBase(channel_name,depth){}
    virtual ~Writer() {
        Shutdown();
    };

public:
    bool Init() override;

    void Shutdown() override;

    virtual bool Write(const MessageT& msg);

    virtual bool Write(const std::shared_ptr<MessageT>& msg_ptr);
};

template <typename MessageT>
bool Writer<MessageT>::Write(const MessageT& msg) {
    auto msg_ptr = std::make_shared<MessageT>(msg);
    return Write(msg_ptr);
}

template <typename MessageT>
bool Writer<MessageT>::Write(const std::shared_ptr<MessageT>& msg_ptr){
    RETURN_VAL_IF(!WriterBase::IsInit(), false);

    unsigned int datalen = ByteSizeLong(*msg_ptr);
    uint8_t *buf = new uint8_t[datalen];

    bool status = false;

    if(!SerializeToArray<MessageT>(*msg_ptr, buf, datalen)){
        AERROR << "serialize to array failed.";
    }
    else {
        status = (lcm_ptr_->publish(channel_name_, buf, datalen) >= 0);
    }
    delete[] buf;
    return status;
}

template<typename MessageT>
bool Writer<MessageT>::Init() {
    if(init_.exchange(true)){
        return true;
    }

    lcm_ptr_ = std::make_unique<lcm::LCM>();

    if((lcm_ptr_ == nullptr) || (!lcm_ptr_->good())){
        init_.store(false);
        return false;
    }
    return true;
}

template<typename MessageT>
void Writer<MessageT>::Shutdown() {
    if(!init_.exchange(false)){
        return;
    }
    lcm_ptr_ = nullptr;
}

} // namespace spider


#endif
