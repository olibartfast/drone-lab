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
};

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
    if (options.backend == "px4") {
      config.update_rate_hz = 10.0;
      config.position_tolerance_m = 0.5;
      config.altitude_tolerance_m = 0.35;
      config.state_timeout_s = 35.0;
      config.mission_timeout_s = 180.0;
    }
    drone_lab::BackyardFlyerMission mission{*vehicle, recorder, config};
    const auto result = mission.run();
    std::cout << "{\"event\":\"summary\",\"backend\":\"" << options.backend
              << "\",\"final_state\":\"" << drone_lab::to_string(result.final_state)
              << "\",\"abort_reason\":\"" << drone_lab::to_string(result.abort_reason)
              << "\",\"ticks\":" << result.ticks << ",\"transitions\":" << result.transitions
              << ",\"endpoint_error_m\":" << result.endpoint_error_m
              << ",\"stale_command_active\":"
              << (result.stale_command_active ? "true" : "false") << "}\n";
    return result.final_state == drone_lab::FlightState::complete ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "{\"event\":\"fatal\",\"message\":\"" << error.what() << "\"}\n";
    return 2;
  }
}
