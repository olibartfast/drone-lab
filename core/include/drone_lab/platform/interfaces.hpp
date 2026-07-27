#pragma once

#include <optional>
#include <string_view>
#include "drone_lab/platform/types.hpp"

namespace drone_lab {
class CameraSource { public: virtual ~CameraSource() = default; [[nodiscard]] virtual std::optional<Frame> acquire_latest_frame() = 0; };
class TelemetrySource { public: virtual ~TelemetrySource() = default; [[nodiscard]] virtual VehicleState latest_state() const = 0; };
class Vehicle {
 public:
  virtual ~Vehicle() = default;
  [[nodiscard]] virtual VehicleState state() const = 0;
  [[nodiscard]] virtual CapabilitySet capabilities() const = 0;
  virtual CommandResult set_velocity(const VelocityCommand&) = 0;
  virtual CommandResult set_gimbal(const GimbalCommand&) = 0;
  virtual CommandResult stop() = 0;
};
class Recorder { public: virtual ~Recorder() = default; virtual void record(std::string_view event_json) = 0; };
}  // namespace drone_lab
