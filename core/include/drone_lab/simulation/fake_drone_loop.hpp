#pragma once

#include <cstdint>
#include <iosfwd>

#include "drone_lab/platform/types.hpp"

namespace drone_lab {

struct FakeDroneScenario {
  std::uint32_t frame_width{640};
  std::uint32_t frame_height{480};
  double update_rate_hz{20.0};
  double duration_s{20.0};
  double target_amplitude_px{180.0};
  double target_frequency_hz{0.05};
  double guidance_gain{0.35};
  double max_yaw_rate_rad_s{0.2617993877991494};
  double target_loss_at_s{-1.0};
  double disconnect_at_s{-1.0};
};

struct FakeDroneMetrics {
  std::uint64_t ticks{0};
  double mean_error_px{0.0};
  double final_error_px{0.0};
  std::uint64_t accepted_commands{0};
  std::uint64_t rejected_commands{0};
  std::uint64_t target_loss_events{0};
  std::uint64_t failsafe_events{0};
  bool unsafe_command_accepted{false};
};

class FakeDroneLoop {
 public:
  explicit FakeDroneLoop(FakeDroneScenario scenario = {});
  [[nodiscard]] FakeDroneMetrics run(std::ostream& jsonl_log) const;

 private:
  FakeDroneScenario scenario_;
};

}  // namespace drone_lab
