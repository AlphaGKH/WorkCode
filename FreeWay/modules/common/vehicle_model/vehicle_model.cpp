#include "modules/common/vehicle_model/vehicle_model.h"

#include "spider/common/file.h"

namespace dharma {
namespace common {

void VehicleModel::RearCenteredKinematicBicycleModel(
    const double predicted_time_horizon, const VehicleState &cur_vehicle_state,
    VehicleState *predicted_vehicle_state) {
  // Kinematic bicycle model centered at rear axis center by Euler forward
  // discretization
  // Assume constant control command and constant z axis position
  CHECK_GT(predicted_time_horizon, 0.0);
  double dt = predicted_time_horizon;
  double cur_x = cur_vehicle_state.x();
  double cur_y = cur_vehicle_state.y();
  double cur_z = cur_vehicle_state.z();
  double cur_phi = cur_vehicle_state.heading();
  double cur_v = cur_vehicle_state.linear_velocity();
  double cur_a = cur_vehicle_state.linear_acceleration();
  double next_x = cur_x;
  double next_y = cur_y;
  double next_phi = cur_phi;
  double next_v = cur_v;

  double countdown_time = predicted_time_horizon;
  bool finish_flag = false;
  static constexpr double kepsilon = 1e-8;
  while (countdown_time > kepsilon && !finish_flag) {
    countdown_time -= dt;
    if (countdown_time < kepsilon) {
      dt = countdown_time + dt;
      finish_flag = true;
    }
    double intermidiate_phi =
        cur_phi + 0.5 * dt * cur_v * cur_vehicle_state.kappa();
    next_phi =
        cur_phi + dt * (cur_v + 0.5 * dt * cur_a) * cur_vehicle_state.kappa();
    next_x =
        cur_x + dt * (cur_v + 0.5 * dt * cur_a) * std::cos(intermidiate_phi);
    next_y =
        cur_y + dt * (cur_v + 0.5 * dt * cur_a) * std::sin(intermidiate_phi);

    next_v = cur_v + dt * cur_a;
    cur_x = next_x;
    cur_y = next_y;
    cur_phi = next_phi;
    cur_v = next_v;
  }

  predicted_vehicle_state->set_x(next_x);
  predicted_vehicle_state->set_y(next_y);
  predicted_vehicle_state->set_z(cur_z);
  predicted_vehicle_state->set_heading(next_phi);
  predicted_vehicle_state->set_kappa(cur_vehicle_state.kappa());
  predicted_vehicle_state->set_linear_velocity(next_v);
  predicted_vehicle_state->set_linear_acceleration(
      cur_vehicle_state.linear_acceleration());
}

VehicleState VehicleModel::Predict(const double predicted_time_horizon,
                                   const VehicleState &cur_vehicle_state) {

  VehicleState predicted_vehicle_state;
  RearCenteredKinematicBicycleModel(predicted_time_horizon, cur_vehicle_state,
                                    &predicted_vehicle_state);

  return predicted_vehicle_state;
}

} // namespace common
} // namespace dharma
