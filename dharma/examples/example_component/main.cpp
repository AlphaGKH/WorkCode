#include "example_component/example_component.h"

int main(){

    spider::Init("EXAMPLE_COMPONENT");

    spider::proto::ComponentConfig config;

    spider::proto::ReaderOption reader0_config;

    reader0_config.set_channel("EXAMPLE_PB");
    reader0_config.set_pending_queue_size(10);
    reader0_config.set_time_out(500);

    config.set_name("EXAMPLE_COMPONENT");

    config.mutable_readers()->Add();
    config.mutable_readers(0)->CopyFrom(reader0_config);


    std::shared_ptr<ComponentBase> base = nullptr;

    base.reset(new ExampleComponent());

    if(base == nullptr || !base->Initialize(config)){
        return 0;
    }

    spider::WaitForShutdown();
}

