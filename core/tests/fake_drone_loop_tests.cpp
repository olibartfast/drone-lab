#include <sstream>
#include <stdexcept>

#include "drone_lab/simulation/fake_drone_loop.hpp"

int main() {
  {
    std::ostringstream first;
    std::ostringstream second;
    const drone_lab::FakeDroneLoop loop;
    const auto a = loop.run(first);
    const auto b = loop.run(second);
    if (first.str() != second.str()) return 1;
    if (a.ticks != 400 || b.ticks != 400) return 2;
    if (a.unsafe_command_accepted) return 3;
    if (a.accepted_commands == 0) return 4;
  }

  {
    drone_lab::FakeDroneScenario scenario;
    scenario.target_loss_at_s = 5.0;
    std::ostringstream log;
    const auto metrics = drone_lab::FakeDroneLoop(scenario).run(log);
    if (metrics.target_loss_events != 1) return 5;
    if (metrics.failsafe_events == 0) return 6;
    if (log.str().find("\"target_visible\":false") == std::string::npos) return 7;
  }

  {
    drone_lab::FakeDroneScenario scenario;
    scenario.disconnect_at_s = 5.0;
    std::ostringstream log;
    const auto metrics = drone_lab::FakeDroneLoop(scenario).run(log);
    if (metrics.failsafe_events == 0) return 8;
    if (log.str().find("\"connected\":false") == std::string::npos) return 9;
  }

  try {
    drone_lab::FakeDroneScenario invalid;
    invalid.update_rate_hz = 0.0;
    const drone_lab::FakeDroneLoop loop(invalid);
    (void)loop;
    return 10;
  } catch (const std::invalid_argument&) {
  }

  return 0;
}
