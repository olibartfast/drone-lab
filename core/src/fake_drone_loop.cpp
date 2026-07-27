#include "drone_lab/simulation/fake_drone_loop.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <stdexcept>

namespace drone_lab {
namespace {
constexpr double kPi = 3.14159265358979323846;

void log_event(std::ostream& out, TimestampNs timestamp_ns, std::string_view event,
               double error_px, double yaw_rate, bool connected, bool visible) {
  out << std::fixed << std::setprecision(6)
      << "{\"timestamp_ns\":" << timestamp_ns
      << ",\"event\":\"" << event << "\""
      << ",\"error_px\":" << error_px
      << ",\"yaw_rate_rad_s\":" << yaw_rate
      << ",\"connected\":" << (connected ? "true" : "false")
      << ",\"target_visible\":" << (visible ? "true" : "false")
      << "}\n";
}
}  // namespace

FakeDroneLoop::FakeDroneLoop(FakeDroneScenario scenario) : scenario_(scenario) {
  if (scenario_.frame_width == 0 || scenario_.frame_height == 0 ||
      scenario_.update_rate_hz <= 0.0 || scenario_.duration_s <= 0.0 ||
      scenario_.max_yaw_rate_rad_s <= 0.0) {
    throw std::invalid_argument("invalid fake drone scenario");
  }
}

FakeDroneMetrics FakeDroneLoop::run(std::ostream& jsonl_log) const {
  FakeDroneMetrics metrics;
  const auto ticks = static_cast<std::uint64_t>(std::llround(
      scenario_.duration_s * scenario_.update_rate_hz));
  const double dt = 1.0 / scenario_.update_rate_hz;
  const double image_center = static_cast<double>(scenario_.frame_width) / 2.0;
  double heading_effect_px = 0.0;
  double error_sum = 0.0;
  bool target_loss_logged = false;
  bool failsafe_active = false;

  for (std::uint64_t tick = 0; tick < ticks; ++tick) {
    const double time_s = static_cast<double>(tick) * dt;
    const auto timestamp_ns = static_cast<TimestampNs>(
        std::llround(time_s * 1'000'000'000.0));
    const bool connected = scenario_.disconnect_at_s < 0.0 ||
                           time_s < scenario_.disconnect_at_s;
    const bool visible = scenario_.target_loss_at_s < 0.0 ||
                         time_s < scenario_.target_loss_at_s;
    const double target_x = image_center + scenario_.target_amplitude_px *
        std::sin(2.0 * kPi * scenario_.target_frequency_hz * time_s);
    const double error_px = visible ? target_x - image_center - heading_effect_px : 0.0;
    double proposal = visible ? scenario_.guidance_gain *
        (error_px / image_center) : 0.0;
    const bool proposal_safe = std::isfinite(proposal) &&
        std::abs(proposal) <= scenario_.max_yaw_rate_rad_s;

    double accepted_yaw_rate = 0.0;
    if (connected && visible && proposal_safe) {
      accepted_yaw_rate = proposal;
      ++metrics.accepted_commands;
    } else {
      ++metrics.rejected_commands;
      if ((!connected || !visible) && !failsafe_active) {
        ++metrics.failsafe_events;
        failsafe_active = true;
      }
    }
    if (connected && visible) {
      failsafe_active = false;
    }
    if (!visible && !target_loss_logged) {
      ++metrics.target_loss_events;
      target_loss_logged = true;
    }
    if (!proposal_safe && accepted_yaw_rate != 0.0) {
      metrics.unsafe_command_accepted = true;
    }

    heading_effect_px += accepted_yaw_rate * dt * image_center;
    if (visible) {
      error_sum += std::abs(error_px);
      metrics.final_error_px = std::abs(error_px);
    }
    log_event(jsonl_log, timestamp_ns,
              accepted_yaw_rate == 0.0 ? "command_stopped" : "command_accepted",
              error_px, accepted_yaw_rate, connected, visible);
    ++metrics.ticks;
  }

  metrics.mean_error_px = metrics.ticks == 0 ? 0.0 :
      error_sum / static_cast<double>(metrics.ticks);
  return metrics;
}

}  // namespace drone_lab
