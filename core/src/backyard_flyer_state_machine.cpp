#include "drone_lab/mission/backyard_flyer.hpp"

namespace drone_lab {
namespace {

[[nodiscard]] bool is_abort_event(MissionEventType event) noexcept {
  switch (event) {
    case MissionEventType::telemetry_disconnected:
    case MissionEventType::arm_rejected:
    case MissionEventType::disarm_rejected:
    case MissionEventType::takeoff_rejected:
    case MissionEventType::position_target_rejected:
    case MissionEventType::land_rejected:
    case MissionEventType::timeout:
    case MissionEventType::stale_telemetry:
    case MissionEventType::invalid_telemetry:
    case MissionEventType::command_failed:
    case MissionEventType::abort_requested:
      return true;
    default:
      return false;
  }
}

[[nodiscard]] bool is_self_event(FlightState state, MissionEventType event) noexcept {
  switch (state) {
    case FlightState::ready:
      return event == MissionEventType::vehicle_ready;
    case FlightState::arming:
      return event == MissionEventType::arm_accepted;
    case FlightState::disarming:
      return event == MissionEventType::disarm_requested ||
             event == MissionEventType::disarm_accepted;
    case FlightState::taking_off:
      return event == MissionEventType::takeoff_requested ||
             event == MissionEventType::takeoff_accepted;
    case FlightState::flying_leg_1:
    case FlightState::flying_leg_2:
    case FlightState::flying_leg_3:
    case FlightState::flying_leg_4:
      return event == MissionEventType::position_target_requested ||
             event == MissionEventType::position_target_accepted;
    case FlightState::landing:
      return event == MissionEventType::land_requested ||
             event == MissionEventType::land_accepted ||
             event == MissionEventType::disarm_requested ||
             event == MissionEventType::disarm_accepted;
    default:
      return false;
  }
}

}  // namespace

FlightStateMachine::FlightStateMachine(MissionScenario scenario) : scenario_(scenario) {}

FlightState FlightStateMachine::state() const noexcept { return state_; }

bool FlightStateMachine::terminal() const noexcept {
  return state_ == FlightState::complete || state_ == FlightState::aborted;
}

MissionActionProposal FlightStateMachine::entry_action() const noexcept {
  switch (state_) {
    case FlightState::arming:
      return {MissionActionType::arm, std::nullopt};
    case FlightState::disarming:
      return {MissionActionType::disarm, std::nullopt};
    case FlightState::taking_off:
      return {MissionActionType::takeoff, std::nullopt};
    case FlightState::flying_leg_1:
      return {MissionActionType::position_target, 0};
    case FlightState::flying_leg_2:
      return {MissionActionType::position_target, 1};
    case FlightState::flying_leg_3:
      return {MissionActionType::position_target, 2};
    case FlightState::flying_leg_4:
      return {MissionActionType::position_target, 3};
    case FlightState::landing:
      return {MissionActionType::land, std::nullopt};
    default:
      return {};
  }
}

TransitionResult FlightStateMachine::handle(const MissionEvent& event) noexcept {
  TransitionResult result{state_, state_, event.type, TransitionReason::invalid_source_state,
                          event.timestamp_ns, event.command_sequence, false};
  if (terminal()) {
    result.reason = TransitionReason::terminal_state;
    return result;
  }
  if (last_event_ == event.type && last_event_state_ == state_) {
    result.reason = TransitionReason::duplicate_event;
    return result;
  }
  if (is_abort_event(event.type)) {
    result.next_state = FlightState::aborted;
    result.reason = TransitionReason::abort;
    result.accepted = true;
  } else if (is_self_event(state_, event.type)) {
    result.reason = TransitionReason::accepted;
    result.accepted = true;
  } else {
    switch (state_) {
      case FlightState::disconnected:
        if (event.type == MissionEventType::telemetry_connected) {
          result.next_state = FlightState::ready;
          result.accepted = true;
        }
        break;
      case FlightState::ready:
        if (event.type == MissionEventType::arm_requested) {
          result.next_state = FlightState::arming;
          result.accepted = true;
        }
        break;
      case FlightState::arming:
        if (event.type == MissionEventType::armed) {
          result.next_state = scenario_ == MissionScenario::arm_only
                                  ? FlightState::disarming
                                  : FlightState::taking_off;
          result.accepted = true;
        }
        break;
      case FlightState::disarming:
        if (event.type == MissionEventType::disarmed) {
          result.next_state = FlightState::complete;
          result.accepted = true;
        }
        break;
      case FlightState::taking_off:
        if (event.type == MissionEventType::takeoff_altitude_reached) {
          result.next_state = scenario_ == MissionScenario::takeoff_only
                                  ? FlightState::landing
                                  : FlightState::flying_leg_1;
          result.accepted = true;
        }
        break;
      case FlightState::flying_leg_1:
        if (event.type == MissionEventType::leg_target_reached) {
          result.next_state = scenario_ == MissionScenario::single_leg
                                  ? FlightState::landing
                                  : FlightState::flying_leg_2;
          result.accepted = true;
        }
        break;
      case FlightState::flying_leg_2:
        if (event.type == MissionEventType::leg_target_reached) {
          result.next_state = FlightState::flying_leg_3;
          result.accepted = true;
        }
        break;
      case FlightState::flying_leg_3:
        if (event.type == MissionEventType::leg_target_reached) {
          result.next_state = FlightState::flying_leg_4;
          result.accepted = true;
        }
        break;
      case FlightState::flying_leg_4:
        if (event.type == MissionEventType::leg_target_reached) {
          result.next_state = FlightState::landing;
          result.accepted = true;
        }
        break;
      case FlightState::landing:
        if (event.type == MissionEventType::landed) {
          result.next_state = FlightState::complete;
          result.accepted = true;
        }
        break;
      case FlightState::complete:
      case FlightState::aborted:
        break;
    }
    if (result.accepted) result.reason = TransitionReason::accepted;
  }
  if (result.accepted) {
    last_event_ = event.type;
    last_event_state_ = result.previous_state;
    state_ = result.next_state;
  }
  return result;
}

std::string_view to_string(MissionScenario scenario) noexcept {
  switch (scenario) {
    case MissionScenario::arm_only: return "arm_only";
    case MissionScenario::takeoff_only: return "takeoff_only";
    case MissionScenario::single_leg: return "single_leg";
    case MissionScenario::square: return "square";
  }
  return "unknown";
}

std::string_view to_string(MissionEventType event) noexcept {
  switch (event) {
    case MissionEventType::command_requested: return "command_requested";
    case MissionEventType::telemetry_connected: return "telemetry_connected";
    case MissionEventType::telemetry_disconnected: return "telemetry_disconnected";
    case MissionEventType::vehicle_ready: return "vehicle_ready";
    case MissionEventType::arm_requested: return "arm_requested";
    case MissionEventType::arm_accepted: return "arm_accepted";
    case MissionEventType::arm_rejected: return "arm_rejected";
    case MissionEventType::armed: return "armed";
    case MissionEventType::disarm_requested: return "disarm_requested";
    case MissionEventType::disarm_accepted: return "disarm_accepted";
    case MissionEventType::disarm_rejected: return "disarm_rejected";
    case MissionEventType::disarmed: return "disarmed";
    case MissionEventType::takeoff_requested: return "takeoff_requested";
    case MissionEventType::takeoff_accepted: return "takeoff_accepted";
    case MissionEventType::takeoff_rejected: return "takeoff_rejected";
    case MissionEventType::takeoff_altitude_reached: return "takeoff_altitude_reached";
    case MissionEventType::position_target_requested: return "position_target_requested";
    case MissionEventType::position_target_accepted: return "position_target_accepted";
    case MissionEventType::position_target_rejected: return "position_target_rejected";
    case MissionEventType::leg_target_reached: return "leg_target_reached";
    case MissionEventType::land_requested: return "land_requested";
    case MissionEventType::land_accepted: return "land_accepted";
    case MissionEventType::land_rejected: return "land_rejected";
    case MissionEventType::landed: return "landed";
    case MissionEventType::timeout: return "timeout";
    case MissionEventType::stale_telemetry: return "stale_telemetry";
    case MissionEventType::invalid_telemetry: return "invalid_telemetry";
    case MissionEventType::command_failed: return "command_failed";
    case MissionEventType::abort_requested: return "abort_requested";
  }
  return "unknown";
}

std::string_view to_string(TransitionReason reason) noexcept {
  switch (reason) {
    case TransitionReason::accepted: return "accepted";
    case TransitionReason::abort: return "abort";
    case TransitionReason::invalid_source_state: return "invalid_source_state";
    case TransitionReason::duplicate_event: return "duplicate_event";
    case TransitionReason::terminal_state: return "terminal_state";
  }
  return "unknown";
}

std::string_view to_string(MissionActionType action) noexcept {
  switch (action) {
    case MissionActionType::none: return "none";
    case MissionActionType::arm: return "arm";
    case MissionActionType::disarm: return "disarm";
    case MissionActionType::takeoff: return "takeoff";
    case MissionActionType::position_target: return "position_target";
    case MissionActionType::land: return "land";
    case MissionActionType::stop: return "stop";
  }
  return "unknown";
}

}  // namespace drone_lab
