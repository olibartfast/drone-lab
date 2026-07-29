#include "drone_lab/adapters/px4/px4_flight_vehicle.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

namespace drone_lab {
namespace {

template <typename Result>
[[nodiscard]] std::string result_text(Result result) {
  std::ostringstream output;
  output << result;
  return output.str();
}

[[nodiscard]] CommandResult action_result(mavsdk::Action::Result result) {
  return {result == mavsdk::Action::Result::Success ? CommandStatus::accepted
                                                    : CommandStatus::rejected,
          result_text(result)};
}

[[nodiscard]] bool finite(const Vector3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

[[nodiscard]] TimestampNs now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

}  // namespace

class Px4FlightVehicle::Impl {
 public:
  Impl(std::string connection_url, double connection_timeout_s)
      : mavsdk{mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation}} {
    const auto connection_result = mavsdk.add_any_connection(connection_url);
    if (connection_result != mavsdk::ConnectionResult::Success) {
      throw std::runtime_error("MAVSDK connection failed: " + result_text(connection_result));
    }
    const auto discovered = mavsdk.first_autopilot(connection_timeout_s);
    if (!discovered) throw std::runtime_error("PX4 autopilot discovery timed out");
    system = *discovered;
    telemetry = std::make_unique<mavsdk::Telemetry>(system);
    action = std::make_unique<mavsdk::Action>(system);
    static_cast<void>(telemetry->set_rate_position_velocity_ned(20.0));

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(connection_timeout_s);
    std::optional<std::chrono::steady_clock::time_point> healthy_since;
    while (true) {
      if (!system->is_connected()) throw std::runtime_error("PX4 disconnected before ready");
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("PX4 health readiness timed out");
      }
      const auto pv = telemetry->position_velocity_ned();
      const bool position_valid = std::isfinite(pv.position.north_m) &&
                                  std::isfinite(pv.position.east_m) &&
                                  std::isfinite(pv.position.down_m);
      if (telemetry->health_all_ok() && position_valid) {
        if (!healthy_since) healthy_since = std::chrono::steady_clock::now();
        if (std::chrono::steady_clock::now() - *healthy_since >= std::chrono::seconds{2}) break;
      } else {
        healthy_since.reset();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }
    local_reference = state().pose.position_m;
    global_reference = telemetry->position();
    if (!std::isfinite(global_reference.latitude_deg) ||
        !std::isfinite(global_reference.longitude_deg) ||
        !std::isfinite(global_reference.absolute_altitude_m)) {
      throw std::runtime_error("PX4 global reference is invalid");
    }
  }

  [[nodiscard]] VehicleState state() const {
    VehicleState result;
    result.connection =
        system->is_connected() ? ConnectionState::connected : ConnectionState::disconnected;
    const auto pv = telemetry->position_velocity_ned();
    result.pose.position_m = {pv.position.east_m, pv.position.north_m, -pv.position.down_m};
    result.velocity.linear_mps = {
        pv.velocity.east_m_s, pv.velocity.north_m_s, -pv.velocity.down_m_s};
    result.pose.frame = CoordinateFrame::world_enu;
    result.velocity.frame = CoordinateFrame::world_enu;
    result.armed = telemetry->armed();
    result.airborne = telemetry->in_air();
    result.home_position_valid = telemetry->health_all_ok();
    switch (telemetry->flight_mode()) {
      case mavsdk::Telemetry::FlightMode::Hold:
        result.navigation_mode = NavigationMode::hold;
        break;
      case mavsdk::Telemetry::FlightMode::Offboard:
      case mavsdk::Telemetry::FlightMode::Mission:
      case mavsdk::Telemetry::FlightMode::Takeoff:
        result.navigation_mode = NavigationMode::mission;
        break;
      case mavsdk::Telemetry::FlightMode::Land:
        result.navigation_mode = NavigationMode::landing;
        break;
      case mavsdk::Telemetry::FlightMode::ReturnToLaunch:
        result.navigation_mode = NavigationMode::return_to_home;
        break;
      default:
        result.navigation_mode = NavigationMode::unknown;
        break;
    }
    result.timestamp_ns = now_ns();
    result.pose.timestamp_ns = result.timestamp_ns;
    result.velocity.timestamp_ns = result.timestamp_ns;
    return result;
  }

  mavsdk::Mavsdk mavsdk;
  std::shared_ptr<mavsdk::System> system;
  std::unique_ptr<mavsdk::Telemetry> telemetry;
  std::unique_ptr<mavsdk::Action> action;
  Vector3 local_reference{};
  mavsdk::Telemetry::Position global_reference{};
};

Px4FlightVehicle::Px4FlightVehicle(std::string connection_url, double connection_timeout_s)
    : impl_(std::make_unique<Impl>(std::move(connection_url), connection_timeout_s)) {}
Px4FlightVehicle::~Px4FlightVehicle() = default;
Px4FlightVehicle::Px4FlightVehicle(Px4FlightVehicle&&) noexcept = default;
Px4FlightVehicle& Px4FlightVehicle::operator=(Px4FlightVehicle&&) noexcept = default;

VehicleState Px4FlightVehicle::latest_state() const { return impl_->state(); }

CommandResult Px4FlightVehicle::arm() {
  if (!impl_->system->is_connected()) return {CommandStatus::disconnected, "PX4 disconnected"};
  if (!impl_->telemetry->health_all_ok()) return {CommandStatus::rejected, "PX4 not ready"};
  return action_result(impl_->action->arm());
}

CommandResult Px4FlightVehicle::disarm() {
  if (!impl_->system->is_connected()) return {CommandStatus::disconnected, "PX4 disconnected"};
  return action_result(impl_->action->disarm());
}

CommandResult Px4FlightVehicle::takeoff(double altitude_m) {
  if (!std::isfinite(altitude_m) || altitude_m <= 0.0 || altitude_m > 10.0) {
    return {CommandStatus::rejected, "takeoff altitude outside simulator limit (0, 10] m"};
  }
  const auto altitude_result =
      impl_->action->set_takeoff_altitude(static_cast<float>(altitude_m));
  if (altitude_result != mavsdk::Action::Result::Success) return action_result(altitude_result);
  return action_result(impl_->action->takeoff());
}

CommandResult Px4FlightVehicle::move_to(const Vector3& position_m, double speed_mps) {
  if (!finite(position_m) || !std::isfinite(speed_mps) || speed_mps <= 0.0 || speed_mps > 2.0) {
    return {CommandStatus::rejected, "invalid or out-of-range simulator position command"};
  }
  const auto current = impl_->state().pose.position_m;
  const double dx = position_m.x - current.x;
  const double dy = position_m.y - current.y;
  const double dz = position_m.z - current.z;
  if (std::sqrt(dx * dx + dy * dy + dz * dz) > 10.0) {
    return {CommandStatus::rejected, "position command exceeds 10 m simulator envelope"};
  }

  constexpr double earth_radius_m = 6'378'137.0;
  constexpr double radians_per_degree = 3.14159265358979323846 / 180.0;
  const double east_m = position_m.x - impl_->local_reference.x;
  const double north_m = position_m.y - impl_->local_reference.y;
  const double latitude =
      impl_->global_reference.latitude_deg + north_m / earth_radius_m / radians_per_degree;
  const double longitude =
      impl_->global_reference.longitude_deg +
      east_m /
          (earth_radius_m * std::cos(impl_->global_reference.latitude_deg * radians_per_degree)) /
          radians_per_degree;
  const float altitude = impl_->global_reference.absolute_altitude_m +
                         static_cast<float>(position_m.z - impl_->local_reference.z);
  const auto speed_result = impl_->action->set_current_speed(static_cast<float>(speed_mps));
  if (speed_result != mavsdk::Action::Result::Success) return action_result(speed_result);
  return action_result(impl_->action->goto_location(latitude, longitude, altitude, 0.0F));
}

CommandResult Px4FlightVehicle::land() {
  return action_result(impl_->action->land());
}

CommandResult Px4FlightVehicle::stop_motion() {
  if (impl_->telemetry->in_air()) return action_result(impl_->action->land());
  return {CommandStatus::accepted, "motion stopped"};
}

void Px4FlightVehicle::advance(double dt_s) {
  std::this_thread::sleep_for(std::chrono::duration<double>(dt_s));
}

}  // namespace drone_lab
