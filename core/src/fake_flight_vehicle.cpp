#include "drone_lab/simulation/fake_flight_vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace drone_lab {
namespace {

[[nodiscard]] std::size_t command_index(FakeFlightCommand command) noexcept {
  return static_cast<std::size_t>(command);
}

}  // namespace

FakeFlightVehicle::FakeFlightVehicle(FakeFlightFaults faults) : faults_(faults) {
  state_.connection =
      faults_.start_disconnected ? ConnectionState::disconnected : ConnectionState::connected;
  state_.pose.frame = CoordinateFrame::world_enu;
  state_.velocity.frame = CoordinateFrame::world_enu;
  state_.navigation_mode = NavigationMode::hold;
  state_.battery_percent = 100.0;
  state_.home_position_valid = !faults_.not_ready;
}

VehicleState FakeFlightVehicle::latest_state() const {
  VehicleState result = state_;
  if (faults_.invalid_telemetry_at_s && elapsed_s_ >= *faults_.invalid_telemetry_at_s) {
    result.pose.position_m.x = std::numeric_limits<double>::quiet_NaN();
  }
  if (faults_.invalid_attitude_at_s && elapsed_s_ >= *faults_.invalid_attitude_at_s) {
    result.pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  }
  return result;
}

CommandResult FakeFlightVehicle::accept_or_reject(FakeFlightCommand command) {
  ++command_counts_[command_index(command)];
  if (state_.connection != ConnectionState::connected) {
    return {CommandStatus::disconnected, "vehicle disconnected"};
  }
  if (faults_.reject_command == command) {
    faults_.reject_command.reset();
    return {CommandStatus::rejected, "injected command rejection"};
  }
  return {CommandStatus::accepted, "accepted"};
}

CommandResult FakeFlightVehicle::arm() {
  auto result = accept_or_reject(FakeFlightCommand::arm);
  if (result.status == CommandStatus::accepted && !faults_.freeze_arm_confirmation) {
    state_.armed = true;
  }
  return result;
}

CommandResult FakeFlightVehicle::disarm() {
  auto result = accept_or_reject(FakeFlightCommand::disarm);
  if (result.status == CommandStatus::accepted && !faults_.freeze_disarm_confirmation) {
    state_.armed = false;
  }
  return result;
}

CommandResult FakeFlightVehicle::takeoff(double altitude_m) {
  auto result = accept_or_reject(FakeFlightCommand::takeoff);
  if (result.status == CommandStatus::accepted && !state_.armed) {
    return {CommandStatus::rejected, "takeoff requires armed vehicle"};
  }
  if (result.status == CommandStatus::accepted && !faults_.freeze_takeoff) {
    target_ = state_.pose.position_m;
    target_.z = altitude_m;
    speed_mps_ = 0.8;
    has_target_ = true;
    landing_ = false;
    state_.airborne = true;
    state_.navigation_mode = NavigationMode::mission;
  }
  return result;
}

CommandResult FakeFlightVehicle::move_to(const Vector3& position_m, double speed_mps) {
  auto result = accept_or_reject(FakeFlightCommand::move_to);
  if (result.status == CommandStatus::accepted && !state_.airborne) {
    return {CommandStatus::rejected, "position target requires airborne vehicle"};
  }
  if (result.status == CommandStatus::accepted) {
    target_ = position_m;
    speed_mps_ = speed_mps;
    has_target_ = true;
    landing_ = false;
    state_.navigation_mode = NavigationMode::mission;
  }
  return result;
}

CommandResult FakeFlightVehicle::land() {
  auto result = accept_or_reject(FakeFlightCommand::land);
  if (result.status == CommandStatus::accepted && !faults_.freeze_landing) {
    target_ = state_.pose.position_m;
    target_.z = 0.0;
    speed_mps_ = 0.6;
    has_target_ = true;
    landing_ = true;
    state_.navigation_mode = NavigationMode::landing;
  }
  return result;
}

void FakeFlightVehicle::stop_motion_internal() noexcept {
  has_target_ = false;
  state_.velocity.linear_mps = {};
}

CommandResult FakeFlightVehicle::stop_motion() {
  const auto result = accept_or_reject(FakeFlightCommand::stop);
  stop_motion_internal();
  return result;
}

void FakeFlightVehicle::advance(double dt_s) {
  elapsed_s_ += dt_s;
  if (!faults_.stale_telemetry_at_s || elapsed_s_ < *faults_.stale_telemetry_at_s) {
    state_.timestamp_ns += static_cast<TimestampNs>(dt_s * 1'000'000'000.0);
    state_.pose.timestamp_ns = state_.timestamp_ns;
    state_.velocity.timestamp_ns = state_.timestamp_ns;
  }

  if (faults_.connect_at_s && elapsed_s_ >= *faults_.connect_at_s) {
    state_.connection = ConnectionState::connected;
  }
  if (faults_.disconnect_at_s && elapsed_s_ >= *faults_.disconnect_at_s) {
    state_.connection = ConnectionState::disconnected;
    stop_motion_internal();
    return;
  }
  if (!has_target_ || faults_.freeze_motion) {
    state_.velocity.linear_mps = {};
    return;
  }

  auto& position = state_.pose.position_m;
  const double dx = target_.x - position.x;
  const double dy = target_.y - position.y;
  const double dz = target_.z - position.z;
  const double remaining = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (remaining <= 1e-9) {
    position = target_;
    stop_motion_internal();
  } else {
    const double step = std::min(speed_mps_ * dt_s, remaining);
    const double scale = step / remaining;
    state_.velocity.linear_mps = {dx / remaining * speed_mps_,
                                  dy / remaining * speed_mps_,
                                  dz / remaining * speed_mps_};
    position.x += dx * scale;
    position.y += dy * scale;
    position.z += dz * scale;
    if (step >= remaining) stop_motion_internal();
  }

  if (landing_ && position.z <= 1e-9) {
    position.z = 0.0;
    state_.airborne = false;
    state_.navigation_mode = NavigationMode::hold;
    landing_ = false;
    stop_motion_internal();
  } else if (has_target_ && std::abs(position.z - target_.z) <= 1e-9) {
    state_.navigation_mode = NavigationMode::hold;
  }
}

std::uint64_t FakeFlightVehicle::command_count(FakeFlightCommand command) const noexcept {
  return command_counts_[command_index(command)];
}

std::uint64_t FakeFlightVehicle::total_command_count() const noexcept {
  return std::accumulate(command_counts_.begin(), command_counts_.end(), std::uint64_t{0});
}

}  // namespace drone_lab
