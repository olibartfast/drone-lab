#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace drone_lab {

using TimestampNs = std::int64_t;
using DurationNs = std::int64_t;

enum class CoordinateFrame { unknown, world_enu, world_ned, vehicle_body_flu, vehicle_body_frd, camera_optical };
enum class PixelFormat { gray8, rgb8, bgr8, rgba8, nv12, nv21, yuv420p };

struct Frame {
  std::vector<std::uint8_t> data;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t row_stride{0};
  PixelFormat format{PixelFormat::rgb8};
  TimestampNs timestamp_ns{0};
  std::uint64_t sequence{0};
  [[nodiscard]] bool valid() const noexcept {
    return width > 0 && height > 0 && row_stride > 0 && !data.empty();
  }
};

struct Vector3 { double x{0.0}; double y{0.0}; double z{0.0}; };
struct Quaternion {
  double w{1.0}; double x{0.0}; double y{0.0}; double z{0.0};
  [[nodiscard]] double norm() const noexcept { return std::sqrt(w*w + x*x + y*y + z*z); }
};
struct Pose { Vector3 position_m{}; Quaternion orientation{}; CoordinateFrame frame{CoordinateFrame::unknown}; TimestampNs timestamp_ns{0}; };
struct Velocity { Vector3 linear_mps{}; Vector3 angular_rad_s{}; CoordinateFrame frame{CoordinateFrame::unknown}; TimestampNs timestamp_ns{0}; };

enum class ConnectionState { disconnected, connecting, connected };
enum class NavigationMode { unknown, manual, hold, mission, return_to_home, landing };

struct VehicleState {
  Pose pose{};
  Velocity velocity{};
  ConnectionState connection{ConnectionState::disconnected};
  bool armed{false};
  bool airborne{false};
  std::optional<double> battery_percent{};
  NavigationMode navigation_mode{NavigationMode::unknown};
  std::optional<bool> home_position_valid{};
  TimestampNs timestamp_ns{0};
};

struct VelocityCommand { Vector3 linear_mps{}; double yaw_rate_rad_s{0.0}; CoordinateFrame frame{CoordinateFrame::vehicle_body_frd}; TimestampNs timestamp_ns{0}; };
struct GimbalCommand { double pitch_rate_rad_s{0.0}; double yaw_rate_rad_s{0.0}; TimestampNs timestamp_ns{0}; };

struct CapabilitySet {
  bool velocity_control{false}; bool position_control{false}; bool waypoint_missions{false};
  bool gimbal_control{false}; bool camera_intrinsics{false}; bool obstacle_data{false}; bool native_rth{false};
};

enum class CommandStatus { accepted, unsupported, rejected, disconnected };
struct CommandResult { CommandStatus status{CommandStatus::rejected}; std::string message; };

}  // namespace drone_lab
