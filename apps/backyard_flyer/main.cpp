#include <iostream>
#include <string_view>

#include "drone_lab/mission/backyard_flyer.hpp"
#include "drone_lab/simulation/fake_flight_vehicle.hpp"

namespace {
class StreamRecorder final : public drone_lab::Recorder {
 public:
  explicit StreamRecorder(std::ostream& output) : output_(output) {}
  void record(std::string_view event_json) override { output_ << event_json << '\n'; }
 private:
  std::ostream& output_;
};
}

int main() {
  drone_lab::FakeFlightVehicle vehicle;
  StreamRecorder recorder{std::cout};
  drone_lab::BackyardFlyerMission mission{vehicle, recorder};
  const auto result = mission.run();
  std::cout << "{\"event\":\"summary\",\"final_state\":\""
            << drone_lab::to_string(result.final_state) << "\",\"abort_reason\":\""
            << drone_lab::to_string(result.abort_reason) << "\",\"ticks\":" << result.ticks
            << ",\"transitions\":" << result.transitions
            << ",\"endpoint_error_m\":" << result.endpoint_error_m
            << ",\"stale_command_active\":"
            << (result.stale_command_active ? "true" : "false") << "}\n";
  return result.final_state == drone_lab::FlightState::complete ? 0 : 1;
}
