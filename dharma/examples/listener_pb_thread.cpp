#include <iostream>

#include "spider/node/reader.h"
#include "spider/time/time.h"
#include "spider/spider.h"
#include "spider/init.h"

#include "examples/examples.pb.h"

void funct(std::shared_ptr<examples::proto::DemoPB> msg) {

    std::cout << "delay time: " << (spider::Time::Now().ToSecond() - msg->timestamp()) << std::endl;
    return;

}

class ThreadDemo
{
public:
    ThreadDemo() {}
    ~ThreadDemo(){}
public:
    bool Init();

    bool Proc(const std::shared_ptr<examples::proto::DemoPB>& msg){
        std::cout << "delay: " << (spider::Time::Now().ToSecond() - msg->timestamp()) << std::endl;
        return true;
    }


private:
    std::unique_ptr<spider::Node> node_ = nullptr;
    std::shared_ptr<spider::Reader<examples::proto::DemoPB> > listener_ = nullptr;

};

bool ThreadDemo::Init(){
    node_ = spider::CreateNode("Listener");

    node_->Name();
    listener_ = node_->CreateReader<examples::proto::DemoPB>("EXAMPLE_PB",10, 500);

    // timer task
    spider::Spinner::Instance()->CreateTimerTask("thread_demo", [this](){
        node_->Observe();

        if(listener_->Empty()){
            AWARN << "Message of " << listener_->GetChannelName() << " is not ready!" << std::endl;
            return;
        }

        auto msg = listener_->GetLatestObserved();

        if(msg != nullptr){
            Proc(msg);
        }
        return;

    }, 100);


//    spider::Spinner::Instance()->CreateTask("thread_demo", [this](){
//        node_->Observe();

//        if(listener_->Empty()){
//            AWARN << "Message of " << listener_->GetChannelName() << " is not ready!" << std::endl;
//            return;
//        }

//        auto msg = listener_->GetLatestObserved();

//        if(msg != nullptr){
//            Proc(msg);
//        }
//        return;

//    });

}

int main()
{   
    spider::Init("Listener");

    ThreadDemo td;

    td.Init();

    spider::WaitForShutdown();

    return 0;
}
