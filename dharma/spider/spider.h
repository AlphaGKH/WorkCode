#ifndef SPIDER_SPIDER_H_
#define SPIDER_SPIDER_H_

#include <memory>
#include "spider/node/node.h"
#include "spider/state.h"

namespace spider {

std::unique_ptr<Node> CreateNode(const std::string& node_name, const std::string& name_space = "");

}

#endif
