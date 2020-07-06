#include "control/control.h"

#include "common/mlog/mlog.h"
#include "common/util/file.h"
#include "common/adapters/adapter_manager.h"

#include "control/common/gflags_control.h"

// MPC
#include "control/MPC/MPC_Controller.h"

namespace control
{

Control::~Control()
{
    if (control_thread_ptr_ && control_thread_ptr_->joinable())
    {
        control_thread_ptr_->join();
    }
}

bool Control::Init()
{
    if(!common::util::GetProtoFromFile(FLAGS_control_conf_file, &config_)){
        AERROR << "Control unable to parse control config file " << FLAGS_control_conf_file;
        return false;
    }

    if(!common::adapter::AdapterManager::Initialized()){
        common::adapter::AdapterManager::Init(FLAGS_control_adapter_conf_file);
    }

    // controller construction
    // controlller init
    MpcVehicleControlInitial(2.5, 40, 0.02);

    return true;
}

bool Control::Start()
{
    if (!Init())
    {
        AERROR << "Control has NOT been initiated.";
        return false;
    }

    control_thread_ptr_ = std::make_unique<std::thread>(&Control::ControlThread, this);

    return true;
}

void Control::ControlThread()
{
    const int32_t kSleepTime = FLAGS_control_cycle; // ms
    while (!is_stop_)
    {
        std::this_thread::yield();

        std::this_thread::sleep_for(
                    std::chrono::duration<double, std::milli>(kSleepTime));
        RunOnce();
    }
    return;
}

void Control::RunOnce()
{
    common::adapter::AdapterManager::Observe();

    if(common::adapter::AdapterManager::GetChassisAdapter()->Empty()){
        AWARN << "Chassis not ready";
        return;
    }

    if(common::adapter::AdapterManager::GetTrajectoryAdapter()->Empty()){
        AWARN << "Planning not ready";
        return;
    }

    const auto chassis = common::adapter::AdapterManager::GetChassisAdapter()->GetLatestObserved();

    const auto traj = common::adapter::AdapterManager::GetTrajectoryAdapter()->GetLatestObserved();

    if(!ControlProcess(chassis, traj)){
        return;
    }


    return;
}

bool Control::ControlProcess(const chassis::Chassis& adc_state, const planning::LcmTrajectory& lcm_traj)
{ 

    if(lcm_traj.num_points < 1){
        return false;
    }

    const double curr_timestamp = common::time::Clock::NowInSeconds();

    double sample_control_steer = 0.0;
    double sample_control_speed = 0.0;

    for(const auto point : lcm_traj.lcm_trajectory){
        if(point.t > 2.0){
//            sample_control_steer = atan(point.kappa * 2.5772);
            sample_control_speed = point.v;
            break;
        }
    }

    int count = 0;

    for(const auto point : lcm_traj.lcm_trajectory){
        if(point.t > 0.5 && point.t < 1.0){
            sample_control_steer += atan(point.kappa * 2.5772);
            count++;
//            sample_control_speed = point.v;
        }
        if(point.t >= 1.0){
            sample_control_steer = sample_control_steer / count;
            break;
        }
    }



//    Trajectory traj;
//    VehiclePosition vehicle_pos;

//    vehicle_pos.x = adc_state.x;
//    vehicle_pos.y = adc_state.y;
//    vehicle_pos.theta = adc_state.theta;
//    vehicle_pos.v = adc_state.v;
//    vehicle_pos.a = adc_state.a;
//    vehicle_pos.kappa = adc_state.kappa;
//    vehicle_pos.t = adc_state.timestamp;

//    traj.showNum = lcm_traj.num_points;
//    traj.pointsNum = lcm_traj.num_points;



//    double zero_theta = 0;
//    double zero_t = lcm_traj.lcm_trajectory.front().t;

//    if(lcm_traj.num_points>0)
//        traj.pathPoint = (VehiclePosition*)malloc(lcm_traj.num_points*sizeof(VehiclePosition));

//    for (size_t i = 0; i < traj.showNum; i++)
//    {
//        traj.pathPoint[i].x = lcm_traj.lcm_trajectory.at(i).x;
//        traj.pathPoint[i].y = lcm_traj.lcm_trajectory.at(i).y;
//        traj.pathPoint[i].theta = lcm_traj.lcm_trajectory.at(i).theta - zero_theta;
//        traj.pathPoint[i].v = lcm_traj.lcm_trajectory.at(i).v;
//        traj.pathPoint[i].a = lcm_traj.lcm_trajectory.at(i).a;
//        traj.pathPoint[i].kappa =lcm_traj.lcm_trajectory.at(i).kappa;
//        traj.pathPoint[i].t = lcm_traj.lcm_trajectory.at(i).t - zero_t;

//    }


//    MpcOutput updatedOutput = MpcVehicleControlProcess(vehicle_pos, traj);

    Command lcm_command_msg;
    lcm_command_msg.timestamp = curr_timestamp;
//    lcm_command_msg.desired_steeringAngle = updatedOutput.frontWheelAngle;
//    lcm_command_msg.desired_velocity = updatedOutput.velocity;
    lcm_command_msg.desired_steeringAngle = sample_control_steer;
    lcm_command_msg.desired_velocity = sample_control_speed;


    common::adapter::AdapterManager::PublishCommand(lcm_command_msg);

    return true;
}

void Control::Stop()
{
    AINFO << "Control Stop is Called!";

    is_stop_ = true;
    if (control_thread_ptr_ && control_thread_ptr_->joinable())
    {
        control_thread_ptr_->join();
    }
}

} // namespace perception
