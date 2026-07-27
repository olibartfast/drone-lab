#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "drone_lab/mission/backyard_flyer.hpp"
#include "drone_lab/simulation/fake_flight_vehicle.hpp"

namespace {
class StringRecorder final : public drone_lab::Recorder {
 public:
  void record(std::string_view event_json) override {
    log_.append(event_json);
    log_.push_back('\n');
  }
  [[nodiscard]] const std::string& log() const { return log_; }
 private:
  std::string log_;
};

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void nominal_square_completes() {
  drone_lab::FakeFlightVehicle vehicle;
  StringRecorder recorder;
  drone_lab::BackyardFlyerMission mission{vehicle, recorder};
  const auto result = mission.run();
  require(result.final_state == drone_lab::FlightState::complete, "nominal mission did not complete");
  require(result.abort_reason == drone_lab::AbortReason::none, "nominal mission aborted");
  require(!result.stale_command_active, "stale command remained active");
  require(result.transitions == 9, "unexpected transition count");
  require(result.endpoint_error_m <= 0.15, "endpoint error too large");
  require(recorder.log().find("\"to\":\"complete\"") != std::string::npos,
          "completion transition not logged");
}

void disconnect_aborts_and_stops() {
  drone_lab::FakeFlightVehicle vehicle{{.disconnect_at_s = 3.0}};
  StringRecorder recorder;
  drone_lab::BackyardFlyerMission mission{vehicle, recorder};
  const auto result = mission.run();
  require(result.final_state == drone_lab::FlightState::aborted, "disconnect did not abort");
  require(result.abort_reason == drone_lab::AbortReason::disconnected, "wrong disconnect reason");
  require(!result.stale_command_active, "disconnect left stale motion");
}

void frozen_motion_times_out() {
  drone_lab::FakeFlightVehicle vehicle{{.freeze_motion = true}};
  StringRecorder recorder;
  drone_lab::BackyardFlyerConfig config;
  config.state_timeout_s = 1.0;
  config.mission_timeout_s = 5.0;
  drone_lab::BackyardFlyerMission mission{vehicle, recorder, config};
  const auto result = mission.run();
  require(result.final_state == drone_lab::FlightState::aborted, "frozen motion did not abort");
  require(result.abort_reason == drone_lab::AbortReason::timeout, "wrong timeout reason");
  require(!result.stale_command_active, "timeout left stale motion");
}

void invalid_config_rejected() {
  drone_lab::FakeFlightVehicle vehicle;
  StringRecorder recorder;
  drone_lab::BackyardFlyerConfig config;
  config.update_rate_hz = 0.0;
  bool threw = false;
  try {
    drone_lab::BackyardFlyerMission mission{vehicle, recorder, config};
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  require(threw, "invalid configuration was accepted");
}
}

int main() {
  try {
    nominal_square_completes();
    disconnect_aborts_and_stops();
    frozen_motion_times_out();
    invalid_config_rejected();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
