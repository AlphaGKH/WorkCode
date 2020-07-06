#ifndef SPIDER_NODE_NODE_H_
#define SPIDER_NODE_NODE_H_

#include <map>

#include "spider/node/writer.h"
#include "spider/node/reader.h"

namespace spider {

template <typename M0, typename M1, typename M2, typename M3>
class Component;

class Node {
public:
    template <typename M0, typename M1, typename M2, typename M3>
    friend class Component;

    friend std::unique_ptr<Node> CreateNode(const std::string&,
                                            const std::string&);
public:
    virtual ~Node();

public:
    const std::string& Name() const;

    template<typename MessageT>
    auto CreateWriter(const std::string& channel_name, const uint32_t& depth = 1) -> std::shared_ptr<Writer<MessageT>>;

    template <typename MessageT>
    auto CreateReader(const std::string& channel_name,  const uint32_t& depth = 1, const uint32_t& time_out = 500,
                     const CallbackFunc<MessageT>& reader_func = nullptr) -> std::shared_ptr<Reader<MessageT>>;

    template <typename MessageT>
    auto CreateReader(const std::string& channel_name,
                     const CallbackFunc<MessageT>& reader_func = nullptr) -> std::shared_ptr<Reader<MessageT>>;

    /**
     * @brief Observe all readers' data
     */
    void Observe();

    /**
     * @brief clear all readers' data
     */
    void ClearData();

private:
    explicit Node(const std::string& node_name,
                  const std::string& name_space = "");

private:
    std::string node_name_;
    std::string name_space_;

    std::mutex readers_mutex_;
    std::map<std::string, std::shared_ptr<ReaderBase>> readers_;
};

template<typename MessageT>
auto Node::CreateWriter(const std::string& channel_name, const uint32_t& depth) -> std::shared_ptr<Writer<MessageT> > {
    std::shared_ptr<Writer<MessageT> >writer_ptr = nullptr;

    writer_ptr = std::make_shared<Writer<MessageT> >(channel_name, depth);

    RETURN_VAL_IF_NULL(writer_ptr, nullptr);
    RETURN_VAL_IF(!writer_ptr->Init(), nullptr);
    return writer_ptr;
}

template <typename MessageT>
auto Node::CreateReader(const std::string& channel_name,  const uint32_t& depth, const uint32_t& time_out,
                        const CallbackFunc<MessageT>& reader_func) -> std::shared_ptr<Reader<MessageT> > {
    std::lock_guard<std::mutex> lg(readers_mutex_);
    if (readers_.find(channel_name) != readers_.end()) {
        AWARN << "Failed to create reader: reader with the same channel already exists.";
        return nullptr;
    }

    std::shared_ptr<Reader<MessageT> > reader_ptr = nullptr;

    reader_ptr = std::make_shared<Reader<MessageT> >(channel_name, reader_func, depth, time_out);

    RETURN_VAL_IF_NULL(reader_ptr, nullptr);
    RETURN_VAL_IF(!reader_ptr->Init(), nullptr);
    readers_.emplace(std::make_pair(channel_name, reader_ptr));

    return reader_ptr;
}

template <typename MessageT>
auto Node::CreateReader(const std::string& channel_name,
                        const CallbackFunc<MessageT>& reader_func) -> std::shared_ptr<Reader<MessageT> > {
    return this->template CreateReader<MessageT>(channel_name, 1, 500, reader_func);
}


} // namespace spider

#endif
