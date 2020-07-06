#include "example_component/example_component.h"

bool ExampleComponent::Init(){
    std::cout << "Example component init!" << std::endl;
    return true;
}

bool ExampleComponent::Proc(const std::shared_ptr<DemoPB> &msg){
    std::cout << "Example component receive a msg: "
              << msg->content() << std::endl;
}
