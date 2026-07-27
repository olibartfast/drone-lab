#pragma once

#include <deque>
#include "drone_lab/platform/interfaces.hpp"

namespace drone_lab {
class FakeCamera final : public CameraSource {
 public:
  void push(Frame frame);
  std::optional<Frame> acquire_latest_frame() override;
 private:
  std::deque<Frame> frames_;
};

class FakeVehicle final : public Vehicle, public TelemetrySource {
 public:
  FakeVehicle();
  VehicleState state() const override;
  VehicleState latest_state() const override;
  CapabilitySet capabilities() const override;
  CommandResult set_velocity(const VelocityCommand& command) override;
  CommandResult set_gimbal(const GimbalCommand& command) override;
  CommandResult stop() override;
  [[nodiscard]] VelocityCommand last_velocity_command() const;
 private:
  VehicleState state_;
  VelocityCommand last_velocity_;
};
}  // namespace drone_lab
