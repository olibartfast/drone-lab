#include "drone_lab/platform/fake_platform.hpp"

#include <utility>

namespace drone_lab {
void FakeCamera::push(Frame frame) { frames_.push_back(std::move(frame)); }

std::optional<Frame> FakeCamera::acquire_latest_frame() {
  if (frames_.empty()) return std::nullopt;
  Frame latest = std::move(frames_.back());
  frames_.clear();
  return latest;
}

FakeVehicle::FakeVehicle() { state_.connection = ConnectionState::connected; }
VehicleState FakeVehicle::state() const { return state_; }
VehicleState FakeVehicle::latest_state() const { return state_; }
CapabilitySet FakeVehicle::capabilities() const {
  return CapabilitySet{.velocity_control = true, .position_control = false, .waypoint_missions = false,
                       .gimbal_control = true, .camera_intrinsics = true, .obstacle_data = false, .native_rth = false};
}
CommandResult FakeVehicle::set_velocity(const VelocityCommand& command) {
  if (state_.connection != ConnectionState::connected) return {CommandStatus::disconnected, "fake vehicle disconnected"};
  last_velocity_ = command;
  return {CommandStatus::accepted, "velocity command accepted"};
}
CommandResult FakeVehicle::set_gimbal(const GimbalCommand&) { return {CommandStatus::accepted, "gimbal command accepted"}; }
CommandResult FakeVehicle::stop() { last_velocity_ = {}; return {CommandStatus::accepted, "vehicle stopped"}; }
VelocityCommand FakeVehicle::last_velocity_command() const { return last_velocity_; }
}  // namespace drone_lab
