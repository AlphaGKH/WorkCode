#include "planning/motion/poly_speed_plan/speed_profile.h"
#include "common/mlog/mlog.h"

namespace planning {

SpeedProfile::SpeedProfile(std::shared_ptr<Curve1d> ptr_curve){
    ptr_speed_profile_ = ptr_curve;
}

double SpeedProfile::Evaluate(const std::uint32_t order, const double param) const{
    double param_length = ptr_speed_profile_->ParamLength();
    if(param < param_length){
        return ptr_speed_profile_->Evaluate(order,param);
    }

    double p = ptr_speed_profile_->Evaluate(0, param_length);
    double v = ptr_speed_profile_->Evaluate(1, param_length);
    double a = ptr_speed_profile_->Evaluate(2, param_length);

    double t = param - param_length;

    switch (order) {
    case 0:
        return p + v * t + 0.5 * a * t * t;
    case 1:
        return v + a * t;
    case 2:
        return a;
    default:
        return 0.0;
    }
}

double SpeedProfile::ParamLength() const{
    return ptr_speed_profile_->ParamLength();
}


std::string SpeedProfile::ToString() const {
    return ptr_speed_profile_->ToString();
}

bool SpeedProfile::has_target_position() const {
    return has_target_position_;
}

bool SpeedProfile::has_target_velocity() const {
    return has_target_velocity_;
}

bool SpeedProfile::has_target_time() const { return has_target_time_; }

double SpeedProfile::target_position() const {
    CHECK(has_target_position_);
    return target_position_;
}

double SpeedProfile::target_velocity() const {
    CHECK(has_target_velocity_);
    return target_velocity_;
}

double SpeedProfile::target_time() const {
    CHECK(has_target_time_);
    return target_time_;
}

void SpeedProfile::set_target_position(double target_position) {
    target_position_ = target_position;
    has_target_position_ = true;
}

void SpeedProfile::set_target_velocity(double target_velocity) {
    target_velocity_ = target_velocity;
    has_target_velocity_ = true;
}

void SpeedProfile::set_target_time(double target_time) {
    target_time_ = target_time;
    has_target_time_ = true;
}

}
