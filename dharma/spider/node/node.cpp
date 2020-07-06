#include "spider/node/node.h"

namespace spider {

Node::Node(const std::string& node_name, const std::string& name_space)
    : node_name_(node_name), name_space_(name_space) {}

Node::~Node() {}

const std::string& Node::Name() const { return node_name_; }

void Node::Observe() {
    for (auto& reader : readers_) {
        reader.second->Observe();
    }
}

void Node::ClearData() {
    for (auto& reader : readers_) {
        reader.second->ClearData();
    }
}

}
