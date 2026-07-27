#pragma once

#include <optional>

#include "drone_lab/mission/backyard_flyer.hpp"

namespace drone_lab {

struct FakeFlightFaults {
  std::optional<double> disconnect_at_s{};
  bool reject_next_command{false};
  bool freeze_motion{false};
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

 private:
  [[nodiscard]] CommandResult accept_or_reject();

  VehicleState state_{};
  FakeFlightFaults faults_{};
  Vector3 target_{};
  double speed_mps_{0.0};
  double elapsed_s_{0.0};
  bool has_target_{false};
  bool landing_{false};
};

}  // namespace drone_lab
