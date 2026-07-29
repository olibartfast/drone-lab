#include "drone_lab/mission/backyard_flyer.hpp"

#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace drone_lab {
namespace {

[[nodiscard]] bool finite(const VehicleState& state) {
  const auto& p = state.pose.position_m;
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

[[nodiscard]] double distance(const Vector3& a, const Vector3& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void record_transition(Recorder& recorder, FlightState from, FlightState to, AbortReason reason,
                       std::uint64_t tick) {
  std::ostringstream out;
  out << "{\"event\":\"transition\",\"tick\":" << tick << ",\"from\":\""
      << to_string(from) << "\",\"to\":\"" << to_string(to)
      << "\",\"reason\":\"" << to_string(reason) << "\"}";
  recorder.record(out.str());
}

void record_telemetry(Recorder& recorder, const VehicleState& state, FlightState flight_state,
                      std::uint64_t tick) {
  std::ostringstream out;
  out << "{\"event\":\"telemetry\",\"tick\":" << tick << ",\"state\":\""
      << to_string(flight_state) << "\",\"frame\":\"world_enu\",\"position_m\":{\"x\":"
      << state.pose.position_m.x << ",\"y\":" << state.pose.position_m.y
      << ",\"z\":" << state.pose.position_m.z << "},\"velocity_mps\":{\"x\":"
      << state.velocity.linear_mps.x << ",\"y\":" << state.velocity.linear_mps.y
      << ",\"z\":" << state.velocity.linear_mps.z << "},\"navigation_mode\":"
      << static_cast<int>(state.navigation_mode) << "}";
  recorder.record(out.str());
}

[[nodiscard]] bool accepted(const CommandResult& result) {
  return result.status == CommandStatus::accepted;
}

}  // namespace

BackyardFlyerMission::BackyardFlyerMission(FlightVehicle& vehicle, Recorder& recorder,
                                           BackyardFlyerConfig config)
    : vehicle_(vehicle), recorder_(recorder), config_(config) {
  if (!(config_.update_rate_hz > 0.0) || !(config_.takeoff_altitude_m > 0.0) ||
      !(config_.leg_length_m > 0.0) || !(config_.horizontal_speed_mps > 0.0) ||
      !(config_.vertical_speed_mps > 0.0) || !(config_.state_timeout_s > 0.0) ||
      !(config_.mission_timeout_s > 0.0) || !(config_.confirmation_time_s > 0.0)) {
    throw std::invalid_argument("BackyardFlyerConfig values must be positive");
  }
}

BackyardFlyerResult BackyardFlyerMission::run() {
  BackyardFlyerResult result{};
  FlightState state = FlightState::disconnected;
  AbortReason abort_reason = AbortReason::none;
  const double dt_s = 1.0 / config_.update_rate_hz;
  double mission_elapsed_s = 0.0;
  double state_elapsed_s = 0.0;
  Vector3 origin{};
  Vector3 target{};
  bool target_issued = false;
  std::uint64_t confirmation_ticks = 0;
  const auto required_confirmation_ticks =
      static_cast<std::uint64_t>(std::ceil(config_.confirmation_time_s * config_.update_rate_hz));

  auto transition = [&](FlightState next, AbortReason reason = AbortReason::none) {
    record_transition(recorder_, state, next, reason, result.ticks);
    state = next;
    abort_reason = reason;
    state_elapsed_s = 0.0;
    target_issued = false;
    confirmation_ticks = 0;
    ++result.transitions;
  };

  auto issue = [&](const CommandResult& command) {
    if (accepted(command)) {
      ++result.commands_accepted;
      return true;
    }
    ++result.commands_rejected;
    return false;
  };

  auto abort = [&](AbortReason reason) {
    static_cast<void>(vehicle_.stop_motion());
    transition(FlightState::aborted, reason);
  };

  while (state != FlightState::complete && state != FlightState::aborted) {
    const VehicleState telemetry = vehicle_.latest_state();
    if (result.ticks % 10 == 0) record_telemetry(recorder_, telemetry, state, result.ticks);
    if (!finite(telemetry)) {
      abort(AbortReason::invalid_telemetry);
      break;
    }
    if (state != FlightState::disconnected && telemetry.connection != ConnectionState::connected) {
      abort(AbortReason::disconnected);
      break;
    }
    if (mission_elapsed_s > config_.mission_timeout_s || state_elapsed_s > config_.state_timeout_s) {
      abort(AbortReason::timeout);
      break;
    }

    switch (state) {
      case FlightState::disconnected:
        if (telemetry.connection == ConnectionState::connected) {
          origin = telemetry.pose.position_m;
          transition(FlightState::ready);
        }
        break;
      case FlightState::ready:
        transition(FlightState::arming);
        break;
      case FlightState::arming:
        if (!target_issued) {
          target_issued = true;
          if (!issue(vehicle_.arm())) {
            abort(AbortReason::command_rejected);
            break;
          }
        }
        if (telemetry.armed) transition(FlightState::taking_off);
        break;
      case FlightState::taking_off:
        if (!target_issued) {
          target_issued = true;
          if (!issue(vehicle_.takeoff(config_.takeoff_altitude_m))) {
            abort(AbortReason::command_rejected);
            break;
          }
        }
        if (telemetry.airborne &&
            telemetry.navigation_mode == NavigationMode::hold &&
            std::abs(telemetry.pose.position_m.z - config_.takeoff_altitude_m) <=
                config_.altitude_tolerance_m &&
            std::abs(telemetry.velocity.linear_mps.z) <= config_.vertical_speed_mps * 0.25) {
          ++confirmation_ticks;
        } else {
          confirmation_ticks = 0;
        }
        if (confirmation_ticks >= required_confirmation_ticks) {
          transition(FlightState::flying_leg_1);
        }
        break;
      case FlightState::flying_leg_1:
      case FlightState::flying_leg_2:
      case FlightState::flying_leg_3:
      case FlightState::flying_leg_4: {
        const std::array<Vector3, 4> targets{{
            {origin.x + config_.leg_length_m, origin.y, config_.takeoff_altitude_m},
            {origin.x + config_.leg_length_m, origin.y + config_.leg_length_m,
             config_.takeoff_altitude_m},
            {origin.x, origin.y + config_.leg_length_m, config_.takeoff_altitude_m},
            {origin.x, origin.y, config_.takeoff_altitude_m},
        }};
        const auto index = static_cast<std::size_t>(state) -
                           static_cast<std::size_t>(FlightState::flying_leg_1);
        target = targets.at(index);
        if (!target_issued) {
          target_issued = true;
          if (!issue(vehicle_.move_to(target, config_.horizontal_speed_mps))) {
            abort(AbortReason::command_rejected);
            break;
          }
        }
        if (distance(telemetry.pose.position_m, target) <= config_.position_tolerance_m) {
          if (state == FlightState::flying_leg_4) {
            result.endpoint_error_m = distance(telemetry.pose.position_m, target);
            transition(FlightState::landing);
          } else {
            transition(static_cast<FlightState>(static_cast<int>(state) + 1));
          }
        }
        break;
      }
      case FlightState::landing:
        if (!target_issued) {
          target_issued = true;
          if (!issue(vehicle_.land())) {
            abort(AbortReason::command_rejected);
            break;
          }
        }
        if (!telemetry.airborne && telemetry.pose.position_m.z <= config_.altitude_tolerance_m) {
          static_cast<void>(vehicle_.stop_motion());
          if (!issue(vehicle_.disarm())) {
            abort(AbortReason::command_rejected);
            break;
          }
          transition(FlightState::complete);
        }
        break;
      case FlightState::complete:
      case FlightState::aborted:
        break;
    }

    vehicle_.advance(dt_s);
    ++result.ticks;
    mission_elapsed_s += dt_s;
    state_elapsed_s += dt_s;
  }

  static_cast<void>(vehicle_.stop_motion());
  result.final_state = state;
  result.abort_reason = abort_reason;
  const VehicleState final_state = vehicle_.latest_state();
  const auto& velocity = final_state.velocity.linear_mps;
  result.stale_command_active =
      std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z) > 0.1;
  return result;
}

std::string_view to_string(FlightState state) noexcept {
  switch (state) {
    case FlightState::disconnected: return "disconnected";
    case FlightState::ready: return "ready";
    case FlightState::arming: return "arming";
    case FlightState::taking_off: return "taking_off";
    case FlightState::flying_leg_1: return "flying_leg_1";
    case FlightState::flying_leg_2: return "flying_leg_2";
    case FlightState::flying_leg_3: return "flying_leg_3";
    case FlightState::flying_leg_4: return "flying_leg_4";
    case FlightState::landing: return "landing";
    case FlightState::complete: return "complete";
    case FlightState::aborted: return "aborted";
  }
  return "unknown";
}

std::string_view to_string(AbortReason reason) noexcept {
  switch (reason) {
    case AbortReason::none: return "none";
    case AbortReason::disconnected: return "disconnected";
    case AbortReason::timeout: return "timeout";
    case AbortReason::command_rejected: return "command_rejected";
    case AbortReason::invalid_telemetry: return "invalid_telemetry";
  }
  return "unknown";
}

}  // namespace drone_lab
