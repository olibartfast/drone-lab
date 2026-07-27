#include "drone_lab/simulation/fake_flight_vehicle.hpp"

#include <algorithm>
#include <cmath>

namespace drone_lab {

FakeFlightVehicle::FakeFlightVehicle(FakeFlightFaults faults) : faults_(faults) {
  state_.connection = ConnectionState::connected;
  state_.pose.frame = CoordinateFrame::world_enu;
  state_.velocity.frame = CoordinateFrame::world_enu;
  state_.navigation_mode = NavigationMode::hold;
  state_.battery_percent = 100.0;
  state_.home_position_valid = true;
}

VehicleState FakeFlightVehicle::latest_state() const { return state_; }

CommandResult FakeFlightVehicle::accept_or_reject() {
  if (state_.connection != ConnectionState::connected) {
    return {CommandStatus::disconnected, "vehicle disconnected"};
  }
  if (faults_.reject_next_command) {
    faults_.reject_next_command = false;
    return {CommandStatus::rejected, "injected command rejection"};
  }
  return {CommandStatus::accepted, "accepted"};
}

CommandResult FakeFlightVehicle::arm() {
  auto result = accept_or_reject();
  if (result.status == CommandStatus::accepted) state_.armed = true;
  return result;
}

CommandResult FakeFlightVehicle::disarm() {
  auto result = accept_or_reject();
  if (result.status == CommandStatus::accepted) state_.armed = false;
  return result;
}

CommandResult FakeFlightVehicle::takeoff(double altitude_m) {
  auto result = accept_or_reject();
  if (result.status == CommandStatus::accepted && state_.armed) {
    target_ = state_.pose.position_m;
    target_.z = altitude_m;
    speed_mps_ = 0.8;
    has_target_ = true;
    landing_ = false;
    state_.airborne = true;
  }
  return result;
}

CommandResult FakeFlightVehicle::move_to(const Vector3& position_m, double speed_mps) {
  auto result = accept_or_reject();
  if (result.status == CommandStatus::accepted && state_.airborne) {
    target_ = position_m;
    speed_mps_ = speed_mps;
    has_target_ = true;
    landing_ = false;
  }
  return result;
}

CommandResult FakeFlightVehicle::land() {
  auto result = accept_or_reject();
  if (result.status == CommandStatus::accepted) {
    target_ = state_.pose.position_m;
    target_.z = 0.0;
    speed_mps_ = 0.6;
    has_target_ = true;
    landing_ = true;
    state_.navigation_mode = NavigationMode::landing;
  }
  return result;
}

CommandResult FakeFlightVehicle::stop_motion() {
  has_target_ = false;
  state_.velocity.linear_mps = {};
  return {CommandStatus::accepted, "stopped"};
}

void FakeFlightVehicle::advance(double dt_s) {
  elapsed_s_ += dt_s;
  state_.timestamp_ns += static_cast<TimestampNs>(dt_s * 1'000'000'000.0);
  state_.pose.timestamp_ns = state_.timestamp_ns;
  state_.velocity.timestamp_ns = state_.timestamp_ns;

  if (faults_.disconnect_at_s && elapsed_s_ >= *faults_.disconnect_at_s) {
    state_.connection = ConnectionState::disconnected;
    static_cast<void>(stop_motion());
    return;
  }
  if (!has_target_ || faults_.freeze_motion) {
    state_.velocity.linear_mps = {};
    return;
  }

  auto& p = state_.pose.position_m;
  const double dx = target_.x - p.x;
  const double dy = target_.y - p.y;
  const double dz = target_.z - p.z;
  const double remaining = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (remaining <= 1e-9) {
    p = target_;
    static_cast<void>(stop_motion());
  } else {
    const double step = std::min(speed_mps_ * dt_s, remaining);
    const double scale = step / remaining;
    state_.velocity.linear_mps = {dx / remaining * speed_mps_, dy / remaining * speed_mps_,
                                  dz / remaining * speed_mps_};
    p.x += dx * scale;
    p.y += dy * scale;
    p.z += dz * scale;
    if (step >= remaining) static_cast<void>(stop_motion());
  }

  if (landing_ && p.z <= 1e-9) {
    p.z = 0.0;
    state_.airborne = false;
    state_.navigation_mode = NavigationMode::hold;
    landing_ = false;
    static_cast<void>(stop_motion());
  }
}

}  // namespace drone_lab
