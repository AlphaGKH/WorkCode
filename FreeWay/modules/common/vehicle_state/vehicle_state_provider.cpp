#include "modules/common/vehicle_state/vehicle_state_provider.h"

#include "spider/common/log.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "modules/common/math/quaternion.h"

namespace dharma {

namespace common {

VehicleStateProvider::VehicleStateProvider() {}

bool VehicleStateProvider::Update(
    const localization::Localization &localization,
    const canbus::Chassis &chassis) {

  if (!ConstructExceptLinearVelocity(localization)) {
    AERROR << "Fail to update because ConstructExceptLinearVelocity error!";
    return false;
  }

  // time_stamp
  if (localization.has_measurement_time()) {
    vehicle_state_.set_timestamp(localization.measurement_time());
  } else if (localization.header().has_timestamp_sec()) {
    vehicle_state_.set_timestamp(localization.header().timestamp_sec());
  } else if (chassis.header().has_timestamp_sec()) {
    AERROR << "Unable to use location timestamp for vehicle state. Use chassis "
              "time instead.";
    vehicle_state_.set_timestamp(chassis.header().timestamp_sec());
  }

  // linear_velocity
  if (chassis.has_speed_mps()) {
    vehicle_state_.set_linear_velocity(chassis.speed_mps());
  }

  // kappa
  static constexpr double kEpsilon = 1e-6;
  if (std::abs(vehicle_state_.linear_velocity()) < kEpsilon) {
    vehicle_state_.set_kappa(0.0);
  } else {
    vehicle_state_.set_kappa(vehicle_state_.angular_velocity() /
                             vehicle_state_.linear_velocity());
  }

  return true;
}

bool VehicleStateProvider::ConstructExceptLinearVelocity(
    const localization::Localization &localization) {
  if (!localization.has_pose()) {
    AERROR << "Invalid localization input.";
    return false;
  }

  vehicle_state_.mutable_pose()->CopyFrom(localization.pose()); // pos

  if (localization.pose().has_position()) {
    vehicle_state_.set_x(localization.pose().position().x()); // x
    vehicle_state_.set_y(localization.pose().position().y()); // y
    vehicle_state_.set_z(localization.pose().position().z()); // z
  }

  const auto &orientation = localization.pose().orientation();

  vehicle_state_.set_heading(common::math::QuaternionToHeading(
      orientation.qw(), orientation.qx(), orientation.qy(),
      orientation.qz())); // heading

  if (!localization.pose().has_angular_velocity_vrf()) {
    AERROR << "localization.pose().has_angular_velocity_vrf() not exists";
    return false;
  }
  vehicle_state_.set_angular_velocity(
      localization.pose().angular_velocity_vrf().z()); // anglar_velocity

  if (!localization.pose().has_linear_acceleration_vrf()) {
    AERROR << "localization.pose().has_linear_acceleration_vrf() not exists";
    return false;
  }
  vehicle_state_.set_linear_acceleration(
      localization.pose().linear_acceleration_vrf().y()); // linear_acc

  return true;
}

math::Vec2d VehicleStateProvider::EstimateFuturePosition(const double t) const {

  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);
  double v = vehicle_state_.linear_velocity();
  // Predict distance travel vector
  if (std::fabs(vehicle_state_.angular_velocity()) < 0.0001) {
    vec_distance[0] = 0.0;
    vec_distance[1] = v * t;
  } else {
    vec_distance[0] = -v / vehicle_state_.angular_velocity() *
                      (1.0 - std::cos(vehicle_state_.angular_velocity() * t));
    vec_distance[1] = std::sin(vehicle_state_.angular_velocity() * t) * v /
                      vehicle_state_.angular_velocity();
  }

  // If we have rotation information, take it into consideration.
  if (vehicle_state_.pose().has_orientation()) {
    const auto &orientation = vehicle_state_.pose().orientation();
    Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(),
                                         orientation.qy(), orientation.qz());

    Eigen::Vector3d pos_vec(vehicle_state_.x(), vehicle_state_.y(),
                            vehicle_state_.z());
    const Eigen::Vector3d future_pos_3d =
        quaternion.toRotationMatrix() * vec_distance + pos_vec;
    return math::Vec2d(future_pos_3d[0], future_pos_3d[1]);
  }

  // If no valid rotation information provided from localization,
  // return the estimated future position without rotation.
  return math::Vec2d(vec_distance[0] + vehicle_state_.x(),
                     vec_distance[1] + vehicle_state_.y());
}

} // namespace common

} // namespace dharma
