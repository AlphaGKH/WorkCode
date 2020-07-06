#include <iostream>

#include "spider/node/reader.h"
#include "spider/time/time.h"
#include "spider/spider.h"
#include "spider/init.h"

#include "examples/examples.pb.h"

void funct(std::shared_ptr<examples::proto::DemoPB> msg) {
//    std::cout << "receive a msg!" << std::endl;
//    std::cout << "msg time: " << msg->timestamp() << std::endl;
    uint64_t now_time = spider::Time::Now().ToNanosecond();
//    std::cout << "receive time: " << now_time << std::endl;

    std::cout << "delay time: " << now_time - msg->timestamp() << std::endl;

}

int main()
{   
    spider::Init("Listener");
    auto listen_node = spider::CreateNode("Listener");

    auto listener = listen_node->CreateReader<examples::proto::DemoPB>("EXAMPLE_PB", &funct);

    listener->Observe();

//    auto msg = listener->GetLatestObserved();

//    std::cout << "msg content: " << msg->content() << std::endl;

    spider::WaitForShutdown();
    return 0;
}
