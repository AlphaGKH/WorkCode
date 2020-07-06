#include "spider/spider.h"

namespace spider {

std::unique_ptr<Node> CreateNode(const std::string& node_name, const std::string& name_space) {

    if(!OK()){
        AERROR << "please initialize spider firstly.";
        return nullptr;
    }

    std::unique_ptr<Node> node(new Node(node_name, name_space));
    return std::move(node);
}


}
