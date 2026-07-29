#include "drone_lab/mission/backyard_flyer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace drone_lab {
namespace {

[[nodiscard]] bool finite(double value) noexcept { return std::isfinite(value); }

[[nodiscard]] bool finite(const Vector3& value) noexcept {
  return finite(value.x) && finite(value.y) && finite(value.z);
}

[[nodiscard]] bool finite(const VehicleState& state) noexcept {
  const auto& orientation = state.pose.orientation;
  return finite(state.pose.position_m) && finite(state.velocity.linear_mps) &&
         finite(state.velocity.angular_rad_s) && finite(orientation.w) &&
         finite(orientation.x) && finite(orientation.y) && finite(orientation.z) &&
         orientation.norm() > 0.0;
}

[[nodiscard]] double distance(const Vector3& a, const Vector3& b) noexcept {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

[[nodiscard]] double horizontal_speed(const VehicleState& state) noexcept {
  const auto& velocity = state.velocity.linear_mps;
  return std::hypot(velocity.x, velocity.y);
}

[[nodiscard]] double cross_track_error(const Vector3& point, const Vector3& start,
                                       const Vector3& target) noexcept {
  const double segment_x = target.x - start.x;
  const double segment_y = target.y - start.y;
  const double length_squared = segment_x * segment_x + segment_y * segment_y;
  if (length_squared <= std::numeric_limits<double>::epsilon()) {
    return std::hypot(point.x - start.x, point.y - start.y);
  }
  const double projection =
      std::clamp(((point.x - start.x) * segment_x + (point.y - start.y) * segment_y) /
                     length_squared,
                 0.0, 1.0);
  return std::hypot(point.x - (start.x + projection * segment_x),
                    point.y - (start.y + projection * segment_y));
}

[[nodiscard]] std::size_t state_index(FlightState state) noexcept {
  return static_cast<std::size_t>(state);
}

[[nodiscard]] std::optional<std::size_t> leg_index(FlightState state) noexcept {
  if (state < FlightState::flying_leg_1 || state > FlightState::flying_leg_4) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(state) -
         static_cast<std::size_t>(FlightState::flying_leg_1);
}

[[nodiscard]] std::string_view command_status_string(CommandStatus status) noexcept {
  switch (status) {
    case CommandStatus::accepted: return "accepted";
    case CommandStatus::unsupported: return "unsupported";
    case CommandStatus::rejected: return "rejected";
    case CommandStatus::disconnected: return "disconnected";
  }
  return "unknown";
}

[[nodiscard]] std::string_view connection_string(ConnectionState state) noexcept {
  switch (state) {
    case ConnectionState::disconnected: return "disconnected";
    case ConnectionState::connecting: return "connecting";
    case ConnectionState::connected: return "connected";
  }
  return "unknown";
}

[[nodiscard]] std::string json_text(std::string_view value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    if (character == '"' || character == '\\') escaped.push_back('\\');
    if (character == '\n' || character == '\r') {
      escaped.push_back(' ');
    } else {
      escaped.push_back(character);
    }
  }
  return escaped;
}

void record_event(Recorder& recorder, std::string_view session_id,
                  const TransitionResult& transition, const VehicleState& telemetry,
                  double telemetry_age_s, MissionActionType command_type,
                  std::optional<std::uint64_t> command_sequence,
                  std::string_view command_result) {
  std::ostringstream out;
  out << "{\"event\":\"mission_event\",\"session_id\":\"" << json_text(session_id)
      << "\",\"timestamp_ns\":" << transition.timestamp_ns
      << ",\"component\":\"backyard_flyer\",\"previous_state\":\""
      << to_string(transition.previous_state) << "\",\"next_state\":\""
      << to_string(transition.next_state) << "\",\"mission_event\":\""
      << to_string(transition.event) << "\",\"transition_accepted\":"
      << (transition.accepted ? "true" : "false") << ",\"transition_reason\":\""
      << to_string(transition.reason) << "\",\"command_type\":\""
      << to_string(command_type) << "\",\"command_sequence\":";
  if (command_sequence) {
    out << *command_sequence;
  } else {
    out << "null";
  }
  out << ",\"command_result\":\"" << json_text(command_result)
      << "\",\"telemetry_age_s\":" << telemetry_age_s
      << ",\"position_m\":{\"x\":" << telemetry.pose.position_m.x
      << ",\"y\":" << telemetry.pose.position_m.y << ",\"z\":"
      << telemetry.pose.position_m.z << "},\"velocity_mps\":{\"x\":"
      << telemetry.velocity.linear_mps.x << ",\"y\":" << telemetry.velocity.linear_mps.y
      << ",\"z\":" << telemetry.velocity.linear_mps.z << "},\"armed_state\":"
      << (telemetry.armed ? "true" : "false") << ",\"landed_state\":"
      << (!telemetry.airborne ? "true" : "false") << ",\"connection_state\":\""
      << connection_string(telemetry.connection) << "\"}";
  recorder.record(out.str());
}

void record_command(Recorder& recorder, std::string_view session_id, FlightState state,
                    const VehicleState& telemetry, double telemetry_age_s,
                    MissionActionType command_type, std::uint64_t command_sequence,
                    const CommandResult& command) {
  TransitionResult observation{state,
                               state,
                               MissionEventType::command_requested,
                               TransitionReason::accepted,
                               telemetry.timestamp_ns,
                               command_sequence,
                               true};
  record_event(recorder, session_id, observation, telemetry, telemetry_age_s, command_type,
               command_sequence, command_status_string(command.status));
}

[[nodiscard]] MissionEventType rejection_event(MissionActionType action) noexcept {
  switch (action) {
    case MissionActionType::arm: return MissionEventType::arm_rejected;
    case MissionActionType::disarm: return MissionEventType::disarm_rejected;
    case MissionActionType::takeoff: return MissionEventType::takeoff_rejected;
    case MissionActionType::position_target: return MissionEventType::position_target_rejected;
    case MissionActionType::land: return MissionEventType::land_rejected;
    case MissionActionType::stop:
    case MissionActionType::none: return MissionEventType::command_failed;
  }
  return MissionEventType::command_failed;
}

[[nodiscard]] MissionEventType accepted_event(MissionActionType action) noexcept {
  switch (action) {
    case MissionActionType::arm: return MissionEventType::arm_accepted;
    case MissionActionType::disarm: return MissionEventType::disarm_accepted;
    case MissionActionType::takeoff: return MissionEventType::takeoff_accepted;
    case MissionActionType::position_target: return MissionEventType::position_target_accepted;
    case MissionActionType::land: return MissionEventType::land_accepted;
    case MissionActionType::stop:
    case MissionActionType::none: return MissionEventType::command_failed;
  }
  return MissionEventType::command_failed;
}

[[nodiscard]] std::optional<MissionEventType> requested_event(
    MissionActionType action) noexcept {
  switch (action) {
    case MissionActionType::disarm: return MissionEventType::disarm_requested;
    case MissionActionType::takeoff: return MissionEventType::takeoff_requested;
    case MissionActionType::position_target:
      return MissionEventType::position_target_requested;
    case MissionActionType::land: return MissionEventType::land_requested;
    case MissionActionType::arm:
    case MissionActionType::stop:
    case MissionActionType::none: return std::nullopt;
  }
  return std::nullopt;
}

[[nodiscard]] double state_timeout(FlightState state,
                                   const BackyardFlyerConfig& config) noexcept {
  switch (state) {
    case FlightState::disconnected: return config.connection_timeout_s;
    case FlightState::ready: return config.ready_timeout_s;
    case FlightState::arming:
    case FlightState::disarming: return config.arm_timeout_s;
    case FlightState::taking_off: return config.takeoff_timeout_s;
    case FlightState::flying_leg_1:
    case FlightState::flying_leg_2:
    case FlightState::flying_leg_3:
    case FlightState::flying_leg_4: return config.leg_timeout_s;
    case FlightState::landing: return config.landing_timeout_s;
    case FlightState::complete:
    case FlightState::aborted: return 0.0;
  }
  return 0.0;
}

}  // namespace

std::array<RelativePositionTarget, kMaximumLegs> make_square_targets(
    const BackyardFlyerConfig& config) {
  return {{{{config.leg_length_m, 0.0, config.takeoff_altitude_m},
            CoordinateFrame::world_enu},
           {{config.leg_length_m, config.leg_length_m, config.takeoff_altitude_m},
            CoordinateFrame::world_enu},
           {{0.0, config.leg_length_m, config.takeoff_altitude_m},
            CoordinateFrame::world_enu},
           {{0.0, 0.0, config.takeoff_altitude_m}, CoordinateFrame::world_enu}}};
}

bool valid_position_target(const RelativePositionTarget& target,
                           const BackyardFlyerConfig& config) noexcept {
  const Vector3 local_origin{};
  return target.frame == CoordinateFrame::world_enu && finite(target.offset_m) &&
         config.maximum_position_target_m > 0.0 &&
         distance(local_origin, target.offset_m) <= config.maximum_position_target_m &&
         config.horizontal_speed_mps > 0.0 &&
         config.horizontal_speed_mps <= config.maximum_horizontal_speed_mps;
}

BackyardFlyerMission::BackyardFlyerMission(FlightVehicle& vehicle, Recorder& recorder,
                                           BackyardFlyerConfig config)
    : vehicle_(vehicle), recorder_(recorder), config_(std::move(config)) {
  const std::array<double, 17> positive_values{
      config_.update_rate_hz,
      config_.takeoff_altitude_m,
      config_.leg_length_m,
      config_.horizontal_speed_mps,
      config_.vertical_speed_mps,
      config_.position_tolerance_m,
      config_.altitude_tolerance_m,
      config_.confirmation_time_s,
      config_.telemetry_max_age_s,
      config_.connection_timeout_s,
      config_.ready_timeout_s,
      config_.arm_timeout_s,
      config_.takeoff_timeout_s,
      config_.leg_timeout_s,
      config_.landing_timeout_s,
      config_.mission_timeout_s,
      config_.maximum_position_target_m,
  };
  if (!std::all_of(positive_values.begin(), positive_values.end(),
                   [](double value) { return finite(value) && value > 0.0; }) ||
      !finite(config_.maximum_horizontal_speed_mps) ||
      !finite(config_.maximum_vertical_speed_mps) ||
      config_.horizontal_speed_mps > config_.maximum_horizontal_speed_mps ||
      config_.vertical_speed_mps > config_.maximum_vertical_speed_mps) {
    throw std::invalid_argument("BackyardFlyerConfig values are invalid or out of bounds");
  }
}

BackyardFlyerResult BackyardFlyerMission::run() {
  BackyardFlyerResult result{};
  result.scenario = config_.scenario;
  FlightStateMachine machine{config_.scenario};
  const double dt_s = 1.0 / config_.update_rate_hz;
  double mission_elapsed_s = 0.0;
  double state_elapsed_s = 0.0;
  double telemetry_age_s = 0.0;
  double confirmation_elapsed_s = 0.0;
  TimestampNs last_telemetry_timestamp = std::numeric_limits<TimestampNs>::min();
  Vector3 origin{};
  const auto relative_targets = make_square_targets(config_);
  std::array<Vector3, kMaximumLegs> targets{};
  bool origin_captured = false;
  bool action_issued = false;
  bool ready_observed = false;
  bool disarm_requested = false;
  std::uint64_t command_sequence = 0;

  auto dispatch = [&](MissionEventType event, const VehicleState& telemetry,
                      std::optional<std::uint64_t> sequence = std::nullopt) {
    const FlightState previous = machine.state();
    const auto transition =
        machine.handle(MissionEvent{event, telemetry.timestamp_ns, sequence});
    record_event(recorder_, config_.session_id, transition, telemetry, telemetry_age_s,
                 MissionActionType::none, sequence, "none");
    if (transition.accepted && transition.next_state != previous) {
      result.metrics.state_durations_s[state_index(previous)] += state_elapsed_s;
      state_elapsed_s = 0.0;
      confirmation_elapsed_s = 0.0;
      action_issued = false;
      disarm_requested = false;
      ++result.transitions;
      if (const auto index = leg_index(transition.next_state)) {
        auto& leg = result.metrics.legs[*index];
        leg.start_position_m = telemetry.pose.position_m;
        leg.target_position_m = targets[*index];
      }
    }
    return transition;
  };

  auto issue = [&](MissionActionType action, const VehicleState& telemetry,
                   const Vector3* position_target = nullptr) {
    ++command_sequence;
    ++result.metrics.command_count;
    if (const auto event = requested_event(action)) {
      dispatch(*event, telemetry, command_sequence);
    }
    CommandResult command;
    switch (action) {
      case MissionActionType::arm: command = vehicle_.arm(); break;
      case MissionActionType::disarm: command = vehicle_.disarm(); break;
      case MissionActionType::takeoff:
        command = vehicle_.takeoff(config_.takeoff_altitude_m);
        break;
      case MissionActionType::position_target:
        if (position_target == nullptr ||
            !valid_position_target(relative_targets[*leg_index(machine.state())], config_)) {
          command = {CommandStatus::rejected, "position target rejected by mission bounds"};
        } else {
          command = vehicle_.move_to(*position_target, config_.horizontal_speed_mps);
        }
        break;
      case MissionActionType::land: command = vehicle_.land(); break;
      case MissionActionType::stop: command = vehicle_.stop_motion(); break;
      case MissionActionType::none:
        command = {CommandStatus::rejected, "invalid empty command"};
        break;
    }
    record_command(recorder_, config_.session_id, machine.state(), telemetry, telemetry_age_s,
                   action, command_sequence, command);
    if (command.status == CommandStatus::accepted) {
      ++result.commands_accepted;
      dispatch(accepted_event(action), telemetry, command_sequence);
      return true;
    }
    ++result.commands_rejected;
    ++result.metrics.command_rejection_count;
    return false;
  };

  auto abort = [&](AbortReason reason, MissionEventType event,
                   const VehicleState& telemetry) {
    if (machine.terminal()) return;
    ++result.metrics.safety_event_count;
    if (reason == AbortReason::timeout) ++result.metrics.timeout_count;
    if (reason == AbortReason::disconnected) ++result.metrics.connection_loss_count;
    const MissionActionType safe_action =
        telemetry.airborne ? MissionActionType::stop
                           : (telemetry.armed ? MissionActionType::disarm
                                              : MissionActionType::stop);
    ++command_sequence;
    ++result.metrics.command_count;
    const CommandResult safe_command =
        safe_action == MissionActionType::disarm ? vehicle_.disarm()
                                                 : vehicle_.stop_motion();
    record_command(recorder_, config_.session_id, machine.state(), telemetry, telemetry_age_s,
                   safe_action, command_sequence, safe_command);
    if (safe_command.status == CommandStatus::accepted) {
      ++result.commands_accepted;
    } else {
      ++result.commands_rejected;
      ++result.metrics.command_rejection_count;
    }
    dispatch(event, telemetry);
    result.abort_reason = reason;
  };

  while (!machine.terminal()) {
    const VehicleState telemetry = vehicle_.latest_state();
    if (last_telemetry_timestamp == std::numeric_limits<TimestampNs>::min() ||
        telemetry.timestamp_ns != last_telemetry_timestamp) {
      last_telemetry_timestamp = telemetry.timestamp_ns;
      telemetry_age_s = 0.0;
    } else {
      telemetry_age_s += dt_s;
    }

    if (!finite(telemetry)) {
      abort(AbortReason::invalid_telemetry, MissionEventType::invalid_telemetry, telemetry);
      break;
    }
    if (telemetry_age_s > config_.telemetry_max_age_s) {
      abort(AbortReason::stale_telemetry, MissionEventType::stale_telemetry, telemetry);
      break;
    }
    if (machine.state() != FlightState::disconnected &&
        telemetry.connection != ConnectionState::connected) {
      abort(AbortReason::disconnected, MissionEventType::telemetry_disconnected, telemetry);
      break;
    }
    if (mission_elapsed_s >= config_.mission_timeout_s ||
        state_elapsed_s >= state_timeout(machine.state(), config_)) {
      abort(AbortReason::timeout, MissionEventType::timeout, telemetry);
      break;
    }

    result.metrics.maximum_horizontal_speed_mps =
        std::max(result.metrics.maximum_horizontal_speed_mps, horizontal_speed(telemetry));
    result.metrics.maximum_vertical_speed_mps =
        std::max(result.metrics.maximum_vertical_speed_mps,
                 std::abs(telemetry.velocity.linear_mps.z));
    if (std::abs(telemetry.velocity.linear_mps.z) >
        config_.maximum_vertical_speed_mps + 1e-6) {
      ++result.metrics.vertical_speed_limit_violations;
    }
    if (origin_captured && machine.state() >= FlightState::taking_off &&
        machine.state() <= FlightState::landing) {
      result.metrics.maximum_altitude_error_m =
          std::max(result.metrics.maximum_altitude_error_m,
                   std::abs(telemetry.pose.position_m.z -
                            (origin.z + config_.takeoff_altitude_m)));
    }
    if (const auto index = leg_index(machine.state())) {
      auto& leg = result.metrics.legs[*index];
      leg.duration_s += dt_s;
      leg.maximum_cross_track_error_m =
          std::max(leg.maximum_cross_track_error_m,
                   cross_track_error(telemetry.pose.position_m, leg.start_position_m,
                                     leg.target_position_m));
      leg.maximum_altitude_deviation_m =
          std::max(leg.maximum_altitude_deviation_m,
                   std::abs(telemetry.pose.position_m.z - leg.target_position_m.z));
      leg.maximum_speed_mps = std::max(leg.maximum_speed_mps, horizontal_speed(telemetry));
    }

    switch (machine.state()) {
      case FlightState::disconnected:
        if (telemetry.connection == ConnectionState::connected) {
          origin = telemetry.pose.position_m;
          for (std::size_t index = 0; index < targets.size(); ++index) {
            const auto& offset = relative_targets[index].offset_m;
            targets[index] = {origin.x + offset.x, origin.y + offset.y, origin.z + offset.z};
          }
          origin_captured = true;
          result.metrics.connection_latency_s = mission_elapsed_s;
          dispatch(MissionEventType::telemetry_connected, telemetry);
        }
        break;
      case FlightState::ready:
        if (telemetry.home_position_valid.value_or(false)) {
          if (!ready_observed) {
            ready_observed = true;
            result.metrics.readiness_latency_s = mission_elapsed_s;
            dispatch(MissionEventType::vehicle_ready, telemetry);
          }
          result.metrics.arm_request_latency_s =
              mission_elapsed_s - result.metrics.readiness_latency_s;
          dispatch(MissionEventType::arm_requested, telemetry);
        }
        break;
      case FlightState::arming:
        if (!action_issued) {
          action_issued = true;
          if (!issue(MissionActionType::arm, telemetry)) {
            abort(AbortReason::command_rejected,
                  rejection_event(MissionActionType::arm), telemetry);
          }
        } else if (telemetry.armed) {
          result.metrics.arm_confirmation_latency_s =
              mission_elapsed_s - result.metrics.readiness_latency_s;
          dispatch(MissionEventType::armed, telemetry);
        }
        break;
      case FlightState::disarming:
        if (!action_issued) {
          action_issued = true;
          if (!issue(MissionActionType::disarm, telemetry)) {
            abort(AbortReason::command_rejected,
                  rejection_event(MissionActionType::disarm), telemetry);
          }
        } else if (!telemetry.armed) {
          dispatch(MissionEventType::disarmed, telemetry);
        }
        break;
      case FlightState::taking_off: {
        if (!action_issued) {
          action_issued = true;
          result.metrics.takeoff_request_time_s = mission_elapsed_s;
          if (!issue(MissionActionType::takeoff, telemetry)) {
            abort(AbortReason::command_rejected,
                  rejection_event(MissionActionType::takeoff), telemetry);
          }
          break;
        }
        const double altitude_error =
            std::abs(telemetry.pose.position_m.z -
                     (origin.z + config_.takeoff_altitude_m));
        if (telemetry.airborne && altitude_error <= config_.altitude_tolerance_m &&
            std::abs(telemetry.velocity.linear_mps.z) <=
                config_.maximum_vertical_speed_mps * 0.25) {
          confirmation_elapsed_s += dt_s;
          if (result.metrics.time_to_altitude_tolerance_s == 0.0) {
            result.metrics.time_to_altitude_tolerance_s =
                mission_elapsed_s - result.metrics.takeoff_request_time_s;
          }
        } else {
          confirmation_elapsed_s = 0.0;
        }
        if (confirmation_elapsed_s >= config_.confirmation_time_s) {
          result.metrics.final_takeoff_altitude_error_m = altitude_error;
          dispatch(MissionEventType::takeoff_altitude_reached, telemetry);
        }
        break;
      }
      case FlightState::flying_leg_1:
      case FlightState::flying_leg_2:
      case FlightState::flying_leg_3:
      case FlightState::flying_leg_4: {
        const auto index = *leg_index(machine.state());
        auto& leg = result.metrics.legs[index];
        if (!action_issued) {
          action_issued = true;
          if (!issue(MissionActionType::position_target, telemetry, &targets[index])) {
            abort(AbortReason::command_rejected,
                  rejection_event(MissionActionType::position_target), telemetry);
          }
          break;
        }
        if (distance(telemetry.pose.position_m, targets[index]) <=
            config_.position_tolerance_m) {
          confirmation_elapsed_s += dt_s;
        } else {
          confirmation_elapsed_s = 0.0;
        }
        if (confirmation_elapsed_s >= config_.confirmation_time_s) {
          leg.endpoint_error_m = distance(telemetry.pose.position_m, targets[index]);
          leg.completed = true;
          ++result.metrics.completed_legs;
          result.metrics.total_path_time_s += leg.duration_s;
          result.endpoint_error_m = leg.endpoint_error_m;
          if (index == 0) {
            result.metrics.frame_sign_verified =
                telemetry.pose.position_m.x > origin.x &&
                telemetry.pose.position_m.x - origin.x >=
                    config_.leg_length_m - config_.position_tolerance_m;
          }
          if (config_.scenario == MissionScenario::square && index == 3) {
            result.metrics.final_origin_error_m =
                std::hypot(telemetry.pose.position_m.x - origin.x,
                           telemetry.pose.position_m.y - origin.y);
          }
          dispatch(MissionEventType::leg_target_reached, telemetry);
        }
        break;
      }
      case FlightState::landing:
        if (!action_issued) {
          action_issued = true;
          result.metrics.landing_request_time_s = mission_elapsed_s;
          if (!issue(MissionActionType::land, telemetry)) {
            abort(AbortReason::command_rejected,
                  rejection_event(MissionActionType::land), telemetry);
          }
        } else if (!telemetry.airborne && !disarm_requested) {
          result.metrics.touchdown_time_s = mission_elapsed_s;
          result.metrics.landing_duration_s =
              mission_elapsed_s - result.metrics.landing_request_time_s;
          disarm_requested = true;
          if (telemetry.armed) {
            if (!issue(MissionActionType::disarm, telemetry)) {
              abort(AbortReason::command_rejected,
                    rejection_event(MissionActionType::disarm), telemetry);
            }
          }
        } else if (!telemetry.airborne && !telemetry.armed) {
          dispatch(MissionEventType::landed, telemetry);
        }
        break;
      case FlightState::complete:
      case FlightState::aborted:
        break;
    }

    if (!machine.terminal()) {
      vehicle_.advance(dt_s);
      ++result.ticks;
      mission_elapsed_s += dt_s;
      state_elapsed_s += dt_s;
    }
  }

  result.metrics.state_durations_s[state_index(machine.state())] += state_elapsed_s;
  result.metrics.total_duration_s = mission_elapsed_s;
  result.final_state = machine.state();
  const VehicleState final_telemetry = vehicle_.latest_state();
  result.metrics.final_landed = !final_telemetry.airborne;
  result.metrics.final_armed = final_telemetry.armed;
  const auto& final_velocity = final_telemetry.velocity.linear_mps;
  result.stale_command_active =
      std::sqrt(final_velocity.x * final_velocity.x + final_velocity.y * final_velocity.y +
                final_velocity.z * final_velocity.z) > 0.1;
  return result;
}

std::string_view to_string(FlightState state) noexcept {
  switch (state) {
    case FlightState::disconnected: return "disconnected";
    case FlightState::ready: return "ready";
    case FlightState::arming: return "arming";
    case FlightState::disarming: return "disarming";
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
    case AbortReason::stale_telemetry: return "stale_telemetry";
  }
  return "unknown";
}

}  // namespace drone_lab
