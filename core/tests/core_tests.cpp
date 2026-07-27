#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "drone_lab/core.hpp"
#include "drone_lab/logging/session_logger.hpp"
#include "drone_lab/platform/fake_platform.hpp"

int main() {
  if (std::string(drone_lab::version()) != "0.1.0") return 1;

  drone_lab::FakeCamera camera;
  drone_lab::Frame frame;
  frame.width = 2;
  frame.height = 2;
  frame.row_stride = 6;
  frame.data = std::vector<std::uint8_t>(12, 42);
  frame.sequence = 7;
  if (!frame.valid()) return 2;
  camera.push(frame);

  const auto latest = camera.acquire_latest_frame();
  if (!latest.has_value() || latest->sequence != 7) return 3;
  if (camera.acquire_latest_frame().has_value()) return 4;

  drone_lab::FakeVehicle vehicle;
  if (!vehicle.capabilities().velocity_control) return 5;

  drone_lab::VelocityCommand command;
  command.linear_mps.x = 0.25;
  const auto result = vehicle.set_velocity(command);
  if (result.status != drone_lab::CommandStatus::accepted) return 6;
  if (vehicle.last_velocity_command().linear_mps.x != 0.25) return 7;

  vehicle.stop();
  if (vehicle.last_velocity_command().linear_mps.x != 0.0) return 8;

  std::ostringstream log;
  drone_lab::SessionLogger logger(log, "test-session");
  logger.log(42, "core", "info", "smoke", "ok");
  if (log.str().find("\"timestamp_ns\":42") == std::string::npos) return 9;

  std::cout << "drone-lab core tests passed\n";
  return 0;
}
