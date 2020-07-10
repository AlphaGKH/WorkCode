#include <iostream>

#include "spider/node/reader.h"
#include "spider/time/time.h"
#include "spider/spider.h"
#include "spider/init.h"

#include "examples/proto/examples.pb.h"

void funct(std::shared_ptr<examples::proto::DemoPB> msg) {

    std::cout << "delay time: " << (spider::Time::Now().ToSecond() - msg->timestamp())
              << "  " << spider::Time::Now().ToNanosecond() - msg->lidar_timestamp()<< std::endl;
    return;

}

int main()
{   
    spider::Init("Listener");
    auto listen_node = spider::CreateNode("Listener");

    auto listener = listen_node->CreateReader<examples::proto::DemoPB>("EXAMPLE_PB", &funct);

    spider::WaitForShutdown();
    return 0;
}
