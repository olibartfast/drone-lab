#pragma once

#include <memory>
#include <string>

#include "drone_lab/mission/backyard_flyer.hpp"

namespace drone_lab {

class Px4FlightVehicle final : public FlightVehicle {
 public:
  explicit Px4FlightVehicle(std::string connection_url, double connection_timeout_s);
  ~Px4FlightVehicle() override;
  Px4FlightVehicle(const Px4FlightVehicle&) = delete;
  Px4FlightVehicle& operator=(const Px4FlightVehicle&) = delete;
  Px4FlightVehicle(Px4FlightVehicle&&) noexcept;
  Px4FlightVehicle& operator=(Px4FlightVehicle&&) noexcept;

  [[nodiscard]] VehicleState latest_state() const override;
  CommandResult arm() override;
  CommandResult disarm() override;
  CommandResult takeoff(double altitude_m) override;
  CommandResult move_to(const Vector3& position_m, double speed_mps) override;
  CommandResult land() override;
  CommandResult stop_motion() override;
  void advance(double dt_s) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace drone_lab
