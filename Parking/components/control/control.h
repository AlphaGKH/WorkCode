#ifndef CONTROL_CONTROL_H_
#define CONTROL_CONTROL_H_

#include <thread>

#include "chassis/Chassis.hpp"
#include "planning/LcmTrajectory.hpp"

#include "control/proto/control_config.pb.h"

namespace control
{

class Control
{
public:
    Control() = default;
    ~Control();

    bool Start();

    void Stop();

private:
    bool Init();

    void ControlThread();

    void RunOnce();

    bool ControlProcess(const chassis::Chassis& adc_state, const planning::LcmTrajectory& lcm_traj);

private:
    // controller

private:
    bool is_stop_ = false;

    std::unique_ptr<std::thread> control_thread_ptr_;

private:
    ControlConfig config_;
};

} // namespace control

#endif
