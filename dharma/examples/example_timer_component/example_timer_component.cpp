#include "example_timer_component/example_timer_component.h"

bool ExampleTimerComponent::Init(){
    std::cout << "Example component init!" << std::endl;
    return true;
}

bool ExampleTimerComponent::Proc(const std::shared_ptr<DemoPB> &msg0,const std::shared_ptr<DemoLCM> &msg1){

    std::cout << "msg0 delay: " << spider::Time::Now().ToSecond() - msg0->timestamp() << std::endl;
    std::cout << "msg1 delay: " << spider::Time::Now().ToSecond() - msg1->timestamp << std::endl;

    return true;
}
