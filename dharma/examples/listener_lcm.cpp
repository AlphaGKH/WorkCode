#include <iostream>

#include "spider/node/reader.h"
#include "spider/time/time.h"
#include "spider/spider.h"
#include "spider/init.h"

#include "examples/lcm/DemoLCM.hpp"

void funct(std::shared_ptr<examples::DemoLCM> msg) {
//    std::cout << "receive a msg!" << std::endl;
    std::cout << "msg time: " << msg->timestamp << std::endl;
    uint64_t now_time = spider::Time::Now().ToNanosecond();
    std::cout << "receive time: " << now_time << std::endl;

    std::cout << "delay time: " << now_time - msg->timestamp << std::endl;

}

int main()
{   
    spider::Init("Listener");
    auto listen_node = spider::CreateNode("Listener");

    auto listener = listen_node->CreateReader<examples::DemoLCM>("EXAMPLE_LCM", &funct);

    spider::WaitForShutdown();

    return 0;
}
