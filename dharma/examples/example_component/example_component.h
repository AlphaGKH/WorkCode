#include "spider/component/component.h"

#include "spider/init.h"

#include "examples/examples.pb.h"

using spider::Component;
using spider::ComponentBase;
using examples::proto::DemoPB;

class ExampleComponent : public Component<DemoPB>{
public:
    bool Init() override;
    bool Proc(const std::shared_ptr<DemoPB> &msg) override;
};

int main(){

    spider::Init("EXAMPLE_COMPONENT");

    spider::proto::ComponentConfig config;

    spider::proto::ReaderOption reader_config;

    reader_config.set_channel("EXAMPLE_PB");
    reader_config.set_pending_queue_size(2);
    reader_config.set_time_out(500);


    config.set_name("EXAMPLE_COMPONENT");
    config.set_flag_file_path("");
    config.set_config_file_path("");

    config.mutable_readers()->Add();
    config.mutable_readers(0)->CopyFrom(reader_config);

    std::shared_ptr<ComponentBase> base = nullptr;

    base.reset(new ExampleComponent());

    if(base == nullptr || !base->Initialize(config)){
        return false;
    }

    spider::WaitForShutdown();

    std::cout << "ExampleComponent Processing!" << std::endl;
}
