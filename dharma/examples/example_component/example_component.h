#include "spider/component/component.h"

#include "spider/init.h"

#include "examples/proto/examples.pb.h"

using spider::Component;
using spider::ComponentBase;
using examples::proto::DemoPB;

class ExampleComponent : public Component<DemoPB>{
public:
    bool Init() override;
    bool Proc(const std::shared_ptr<DemoPB> &msg0) override;
};
