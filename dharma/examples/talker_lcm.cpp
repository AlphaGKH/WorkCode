#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "spider/node/writer.h"
#include "spider/time/rate.h"
#include "spider/spider.h"

#include "examples/lcm/DemoLCM.hpp"

int main()
{
    spider::Init("Talker");

    auto talk_node = spider::CreateNode("Talker");

    auto talker = talk_node->CreateWriter<examples::DemoLCM>("EXAMPLE_LCM");

    if(talker == nullptr){
        return 0;
    }

    spider::Rate rate(100.0);

    while (true) {
        static size_t count = 0;
        auto msg = std::make_shared<examples::DemoLCM>();
        msg->timestamp = spider::Time::Now().ToNanosecond();
        msg->lidar_timestamp = spider::Time::Now().ToNanosecond();
        msg->seq = count++;
        msg->context = "lcm communication";

        talker->Write(msg);

        std::cout << "Send a Message!" << std::endl;

        rate.Sleep();
    }
    return 0;
}
