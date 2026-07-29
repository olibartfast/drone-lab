#include <array>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>

#include "drone_lab/mission/backyard_flyer.hpp"
#include "drone_lab/simulation/fake_flight_vehicle.hpp"
#if defined(DRONE_LAB_HAS_PX4_SIM)
#include "drone_lab/adapters/px4/px4_flight_vehicle.hpp"
#endif

namespace {
class StreamRecorder final : public drone_lab::Recorder {
 public:
  explicit StreamRecorder(std::ostream& output) : output_(output) {}
  void record(std::string_view event_json) override { output_ << event_json << '\n'; }

 private:
  std::ostream& output_;
};

struct Options {
  std::string backend{"fake"};
  std::string connection{"udpin://0.0.0.0:14540"};
  double connection_timeout_s{60.0};
  drone_lab::MissionScenario scenario{drone_lab::MissionScenario::square};
};

[[nodiscard]] drone_lab::MissionScenario parse_scenario(std::string_view value) {
  if (value == "arm-only") return drone_lab::MissionScenario::arm_only;
  if (value == "takeoff-only") return drone_lab::MissionScenario::takeoff_only;
  if (value == "single-leg") return drone_lab::MissionScenario::single_leg;
  if (value == "square") return drone_lab::MissionScenario::square;
  throw std::invalid_argument(
      "--scenario must be arm-only, takeoff-only, single-leg, or square");
}

[[nodiscard]] Options parse_options(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument{argv[index]};
    const auto value = [&](std::string_view name) -> std::string_view {
      if (index + 1 >= argc) throw std::invalid_argument(std::string{name} + " requires a value");
      return argv[++index];
    };
    if (argument == "--backend") {
      options.backend = value("--backend");
    } else if (argument == "--connection") {
      options.connection = value("--connection");
    } else if (argument == "--connection-timeout") {
      options.connection_timeout_s = std::stod(std::string{value("--connection-timeout")});
    } else if (argument == "--scenario") {
      options.scenario = parse_scenario(value("--scenario"));
    } else {
      throw std::invalid_argument("unknown argument: " + std::string{argument});
    }
  }
  if (options.backend != "fake" && options.backend != "px4") {
    throw std::invalid_argument("--backend must be fake or px4");
  }
  if (!(options.connection_timeout_s > 0.0)) {
    throw std::invalid_argument("--connection-timeout must be positive");
  }
  return options;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_options(argc, argv);
    std::unique_ptr<drone_lab::FlightVehicle> vehicle;
    if (options.backend == "fake") {
      vehicle = std::make_unique<drone_lab::FakeFlightVehicle>();
    } else {
#if defined(DRONE_LAB_HAS_PX4_SIM)
      vehicle = std::make_unique<drone_lab::Px4FlightVehicle>(
          options.connection, options.connection_timeout_s);
#else
      throw std::runtime_error(
          "PX4 backend unavailable: configure with -DDRONE_LAB_ENABLE_PX4_SIM=ON");
#endif
    }

    StreamRecorder recorder{std::cout};
    drone_lab::BackyardFlyerConfig config;
    config.scenario = options.scenario;
    if (options.backend == "px4") {
      config.update_rate_hz = 10.0;
      config.position_tolerance_m = 0.5;
      config.altitude_tolerance_m = 0.35;
      config.confirmation_time_s = 0.8;
      config.ready_timeout_s = 15.0;
      config.arm_timeout_s = 20.0;
      config.takeoff_timeout_s = 35.0;
      config.leg_timeout_s = 35.0;
      config.landing_timeout_s = 35.0;
      config.mission_timeout_s = 180.0;
    }
    drone_lab::BackyardFlyerMission mission{*vehicle, recorder, config};
    const auto result = mission.run();
    const auto& metrics = result.metrics;
    const std::array<drone_lab::FlightState, drone_lab::kFlightStateCount> states{
        drone_lab::FlightState::disconnected, drone_lab::FlightState::ready,
        drone_lab::FlightState::arming,       drone_lab::FlightState::disarming,
        drone_lab::FlightState::taking_off,   drone_lab::FlightState::flying_leg_1,
        drone_lab::FlightState::flying_leg_2, drone_lab::FlightState::flying_leg_3,
        drone_lab::FlightState::flying_leg_4, drone_lab::FlightState::landing,
        drone_lab::FlightState::complete,     drone_lab::FlightState::aborted};
    std::cout << "{\"event\":\"summary\",\"backend\":\"" << options.backend
              << "\",\"scenario\":\"" << drone_lab::to_string(result.scenario)
              << "\",\"terminal_state\":\"" << drone_lab::to_string(result.final_state)
              << "\",\"final_state\":\"" << drone_lab::to_string(result.final_state)
              << "\",\"abort_reason\":\"" << drone_lab::to_string(result.abort_reason)
              << "\",\"ticks\":" << result.ticks << ",\"transitions\":" << result.transitions
              << ",\"endpoint_error_m\":" << result.endpoint_error_m
              << ",\"total_duration_s\":" << metrics.total_duration_s
              << ",\"state_durations_s\":{";
    for (std::size_t index = 0; index < states.size(); ++index) {
      if (index != 0) std::cout << ',';
      std::cout << '"' << drone_lab::to_string(states[index]) << "\":"
                << metrics.state_durations_s[index];
    }
    std::cout << "},\"command_count\":" << metrics.command_count
              << ",\"command_rejection_count\":" << metrics.command_rejection_count
              << ",\"timeout_count\":" << metrics.timeout_count
              << ",\"connection_loss_count\":" << metrics.connection_loss_count
              << ",\"safety_event_count\":" << metrics.safety_event_count
              << ",\"maximum_horizontal_speed_mps\":"
              << metrics.maximum_horizontal_speed_mps
              << ",\"maximum_vertical_speed_mps\":" << metrics.maximum_vertical_speed_mps
              << ",\"maximum_altitude_error_m\":" << metrics.maximum_altitude_error_m
              << ",\"connection_latency_s\":" << metrics.connection_latency_s
              << ",\"readiness_latency_s\":" << metrics.readiness_latency_s
              << ",\"arm_confirmation_latency_s\":" << metrics.arm_confirmation_latency_s
              << ",\"takeoff_request_time_s\":" << metrics.takeoff_request_time_s
              << ",\"time_to_altitude_tolerance_s\":"
              << metrics.time_to_altitude_tolerance_s
              << ",\"final_takeoff_altitude_error_m\":"
              << metrics.final_takeoff_altitude_error_m
              << ",\"vertical_speed_limit_violations\":"
              << metrics.vertical_speed_limit_violations
              << ",\"completed_legs\":" << metrics.completed_legs
              << ",\"per_leg_endpoint_error_m\":[";
    for (std::size_t index = 0; index < metrics.legs.size(); ++index) {
      if (index != 0) std::cout << ',';
      std::cout << metrics.legs[index].endpoint_error_m;
    }
    std::cout << "],\"per_leg_cross_track_error_m\":[";
    for (std::size_t index = 0; index < metrics.legs.size(); ++index) {
      if (index != 0) std::cout << ',';
      std::cout << metrics.legs[index].maximum_cross_track_error_m;
    }
    std::cout << "],\"total_path_time_s\":" << metrics.total_path_time_s
              << ",\"final_origin_error_m\":" << metrics.final_origin_error_m
              << ",\"frame_sign_verified\":"
              << (metrics.frame_sign_verified ? "true" : "false")
              << ",\"landing_request_time_s\":" << metrics.landing_request_time_s
              << ",\"touchdown_time_s\":" << metrics.touchdown_time_s
              << ",\"landing_duration_s\":" << metrics.landing_duration_s
              << ",\"landed\":" << (metrics.final_landed ? "true" : "false")
              << ",\"armed\":" << (metrics.final_armed ? "true" : "false")
              << ",\"stale_command_active\":"
              << (result.stale_command_active ? "true" : "false") << "}\n";
    return result.final_state == drone_lab::FlightState::complete ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "{\"event\":\"fatal\",\"message\":\"" << error.what() << "\"}\n";
    return 2;
  }
}
