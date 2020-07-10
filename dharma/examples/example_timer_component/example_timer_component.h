#include "spider/component/timer_component.h"

#include "spider/init.h"

#include "examples/proto/examples.pb.h"
#include "examples/lcm/DemoLCM.hpp"

using spider::TimerComponent;
using spider::ComponentBase;
using examples::proto::DemoPB;
using examples::DemoLCM;

class ExampleTimerComponent : public TimerComponent<DemoPB, DemoLCM>{
public:
    bool Init() override;
    bool Proc(const std::shared_ptr<DemoPB> &msg0,
              const std::shared_ptr<DemoLCM> &msg1) override;
};
