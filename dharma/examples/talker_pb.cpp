#include <iostream>

#include <lcm/lcm-cpp.hpp>
#include "spider/node/writer.h"
#include "spider/time/rate.h"
#include "spider/spider.h"
#include "spider/init.h"

#include "examples/examples.pb.h"

int main()
{
    spider::Init("Talker");
    auto talk_node = spider::CreateNode("Talker");

    auto talker = talk_node->CreateWriter<examples::proto::DemoPB>("EXAMPLE_PB");

    if(talker == nullptr){
        return 0;
    }

    spider::Rate rate(10.0);

    while (spider::OK()) {
        static size_t count = 0;
        auto msg = std::make_shared<examples::proto::DemoPB>();
        msg->set_timestamp(spider::Time::Now().ToNanosecond());
        msg->set_lidar_timestamp(spider::Time::Now().ToNanosecond());
        msg->set_seq(count++);
        msg->set_content("proto contents");

        talker->Write(msg);

        std::cout << "Send a Message!" << std::endl;

        rate.Sleep();
    }
    return 0;
}

