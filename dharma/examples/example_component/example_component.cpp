#include "example_component/example_component.h"

bool ExampleComponent::Init(){
    std::cout << "Example component init!" << std::endl;
    return true;
}

bool ExampleComponent::Proc(const std::shared_ptr<DemoPB> &msg0){

    double delay = (spider::Time::Now().ToSecond() - msg0->timestamp());

    std::cout << "msg0 delay: " << delay << std::endl;
    return true;
}
