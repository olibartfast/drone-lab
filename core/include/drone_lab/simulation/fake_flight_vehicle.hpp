#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "drone_lab/mission/backyard_flyer.hpp"

namespace drone_lab {

enum class FakeFlightCommand {
  arm,
  disarm,
  takeoff,
  move_to,
  land,
  stop,
};

struct FakeFlightFaults {
  bool start_disconnected{false};
  std::optional<double> connect_at_s{};
  std::optional<double> disconnect_at_s{};
  std::optional<double> invalid_telemetry_at_s{};
  std::optional<double> invalid_attitude_at_s{};
  std::optional<double> stale_telemetry_at_s{};
  std::optional<FakeFlightCommand> reject_command{};
  bool freeze_motion{false};
  bool freeze_arm_confirmation{false};
  bool freeze_disarm_confirmation{false};
  bool freeze_takeoff{false};
  bool freeze_landing{false};
  bool not_ready{false};
};

class FakeFlightVehicle final : public FlightVehicle {
 public:
  explicit FakeFlightVehicle(FakeFlightFaults faults = {});

  [[nodiscard]] VehicleState latest_state() const override;
  CommandResult arm() override;
  CommandResult disarm() override;
  CommandResult takeoff(double altitude_m) override;
  CommandResult move_to(const Vector3& position_m, double speed_mps) override;
  CommandResult land() override;
  CommandResult stop_motion() override;
  void advance(double dt_s) override;
  [[nodiscard]] std::uint64_t command_count(FakeFlightCommand command) const noexcept;
  [[nodiscard]] std::uint64_t total_command_count() const noexcept;

 private:
  [[nodiscard]] CommandResult accept_or_reject(FakeFlightCommand command);
  void stop_motion_internal() noexcept;

  VehicleState state_{};
  FakeFlightFaults faults_{};
  Vector3 target_{};
  double speed_mps_{0.0};
  double elapsed_s_{0.0};
  bool has_target_{false};
  bool landing_{false};
  std::array<std::uint64_t, 6> command_counts_{};
};

}  // namespace drone_lab
