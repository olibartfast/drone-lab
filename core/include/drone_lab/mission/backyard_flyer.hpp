#pragma once

#include <cstdint>
#include <string_view>

#include "drone_lab/platform/interfaces.hpp"

namespace drone_lab {

enum class FlightState {
  disconnected,
  ready,
  arming,
  taking_off,
  flying_leg_1,
  flying_leg_2,
  flying_leg_3,
  flying_leg_4,
  landing,
  complete,
  aborted,
};

enum class AbortReason {
  none,
  disconnected,
  timeout,
  command_rejected,
  invalid_telemetry,
};

struct BackyardFlyerConfig {
  double update_rate_hz{20.0};
  double takeoff_altitude_m{2.0};
  double leg_length_m{4.0};
  double horizontal_speed_mps{1.0};
  double vertical_speed_mps{0.8};
  double position_tolerance_m{0.15};
  double altitude_tolerance_m{0.10};
  double state_timeout_s{15.0};
  double mission_timeout_s{90.0};
};

struct BackyardFlyerResult {
  FlightState final_state{FlightState::disconnected};
  AbortReason abort_reason{AbortReason::none};
  std::uint64_t ticks{0};
  std::uint64_t transitions{0};
  std::uint64_t commands_accepted{0};
  std::uint64_t commands_rejected{0};
  double endpoint_error_m{0.0};
  bool stale_command_active{false};
};

class FlightVehicle : public TelemetrySource {
 public:
  ~FlightVehicle() override = default;
  virtual CommandResult arm() = 0;
  virtual CommandResult disarm() = 0;
  virtual CommandResult takeoff(double altitude_m) = 0;
  virtual CommandResult move_to(const Vector3& position_m, double speed_mps) = 0;
  virtual CommandResult land() = 0;
  virtual CommandResult stop_motion() = 0;
  virtual void advance(double dt_s) = 0;
};

class BackyardFlyerMission {
 public:
  BackyardFlyerMission(FlightVehicle& vehicle, Recorder& recorder, BackyardFlyerConfig config = {});
  [[nodiscard]] BackyardFlyerResult run();

 private:
  FlightVehicle& vehicle_;
  Recorder& recorder_;
  BackyardFlyerConfig config_;
};

[[nodiscard]] std::string_view to_string(FlightState state) noexcept;
[[nodiscard]] std::string_view to_string(AbortReason reason) noexcept;

}  // namespace drone_lab
