#include <array>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

#include "drone_lab/mission/backyard_flyer.hpp"
#include "drone_lab/simulation/fake_flight_vehicle.hpp"

namespace {

class StringRecorder final : public drone_lab::Recorder {
 public:
  void record(std::string_view event_json) override {
    log_.append(event_json);
    log_.push_back('\n');
  }
  [[nodiscard]] const std::string& log() const noexcept { return log_; }

 private:
  std::string log_;
};

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

[[nodiscard]] drone_lab::BackyardFlyerConfig fast_config(
    drone_lab::MissionScenario scenario = drone_lab::MissionScenario::square) {
  drone_lab::BackyardFlyerConfig config;
  config.scenario = scenario;
  config.update_rate_hz = 20.0;
  config.takeoff_altitude_m = 1.0;
  config.leg_length_m = 1.0;
  config.horizontal_speed_mps = 1.0;
  config.vertical_speed_mps = 0.8;
  config.position_tolerance_m = 0.1;
  config.altitude_tolerance_m = 0.1;
  config.confirmation_time_s = 0.1;
  config.connection_timeout_s = 0.5;
  config.ready_timeout_s = 0.5;
  config.arm_timeout_s = 0.5;
  config.takeoff_timeout_s = 3.0;
  config.leg_timeout_s = 3.0;
  config.landing_timeout_s = 3.0;
  config.mission_timeout_s = 20.0;
  return config;
}

[[nodiscard]] drone_lab::BackyardFlyerResult run(
    drone_lab::MissionScenario scenario, StringRecorder& recorder,
    drone_lab::FakeFlightVehicle& vehicle,
    const drone_lab::BackyardFlyerConfig* override_config = nullptr) {
  auto config = override_config == nullptr ? fast_config(scenario) : *override_config;
  drone_lab::BackyardFlyerMission mission{vehicle, recorder, config};
  return mission.run();
}

void state_machine_contract() {
  using drone_lab::FlightState;
  using drone_lab::MissionEvent;
  using drone_lab::MissionEventType;

  drone_lab::FlightStateMachine invalid{drone_lab::MissionScenario::square};
  auto transition = invalid.handle(MissionEvent{MissionEventType::armed, 1, std::nullopt});
  require(!transition.accepted, "invalid source event was accepted");
  require(transition.reason == drone_lab::TransitionReason::invalid_source_state,
          "invalid transition reason missing");
  transition = invalid.handle(
      MissionEvent{MissionEventType::telemetry_connected, 2, std::nullopt});
  require(transition.accepted && transition.next_state == FlightState::ready,
          "connection transition failed");
  transition = invalid.handle(MissionEvent{MissionEventType::vehicle_ready, 3, std::nullopt});
  require(transition.accepted && transition.next_state == FlightState::ready,
          "ready observation failed");
  transition = invalid.handle(MissionEvent{MissionEventType::vehicle_ready, 4, std::nullopt});
  require(!transition.accepted &&
              transition.reason == drone_lab::TransitionReason::duplicate_event,
          "duplicate event was not rejected");

  struct ScenarioPath {
    drone_lab::MissionScenario scenario;
    std::array<MissionEventType, 9> events;
    std::size_t count;
  };
  const std::array paths{
      ScenarioPath{drone_lab::MissionScenario::arm_only,
                   {MissionEventType::telemetry_connected, MissionEventType::arm_requested,
                    MissionEventType::armed, MissionEventType::disarmed},
                   4},
      ScenarioPath{drone_lab::MissionScenario::takeoff_only,
                   {MissionEventType::telemetry_connected, MissionEventType::arm_requested,
                    MissionEventType::armed, MissionEventType::takeoff_altitude_reached,
                    MissionEventType::landed},
                   5},
      ScenarioPath{drone_lab::MissionScenario::single_leg,
                   {MissionEventType::telemetry_connected, MissionEventType::arm_requested,
                    MissionEventType::armed, MissionEventType::takeoff_altitude_reached,
                    MissionEventType::leg_target_reached, MissionEventType::landed},
                   6},
      ScenarioPath{drone_lab::MissionScenario::square,
                   {MissionEventType::telemetry_connected, MissionEventType::arm_requested,
                    MissionEventType::armed, MissionEventType::takeoff_altitude_reached,
                    MissionEventType::leg_target_reached, MissionEventType::leg_target_reached,
                    MissionEventType::leg_target_reached, MissionEventType::leg_target_reached,
                    MissionEventType::landed},
                   9},
  };
  for (const auto& path : paths) {
    drone_lab::FlightStateMachine machine{path.scenario};
    for (std::size_t index = 0; index < path.count; ++index) {
      transition =
          machine.handle(MissionEvent{path.events[index], static_cast<drone_lab::TimestampNs>(index),
                                      std::nullopt});
      require(transition.accepted, "nominal scenario transition rejected");
    }
    require(machine.state() == FlightState::complete, "scenario path did not complete");
    transition = machine.handle(MissionEvent{MissionEventType::abort_requested, 99, std::nullopt});
    require(!transition.accepted &&
                transition.reason == drone_lab::TransitionReason::terminal_state,
            "event after terminal state was accepted");
    require(machine.entry_action().type == drone_lab::MissionActionType::none,
            "terminal state proposed a command");
  }

  drone_lab::FlightStateMachine actions{drone_lab::MissionScenario::square};
  static_cast<void>(
      actions.handle(MissionEvent{MissionEventType::telemetry_connected, 0, std::nullopt}));
  static_cast<void>(actions.handle(MissionEvent{MissionEventType::arm_requested, 1, std::nullopt}));
  require(actions.entry_action().type == drone_lab::MissionActionType::arm,
          "arming action proposal missing");
  static_cast<void>(actions.handle(MissionEvent{MissionEventType::armed, 2, std::nullopt}));
  require(actions.entry_action().type == drone_lab::MissionActionType::takeoff,
          "takeoff action proposal missing");

  const std::array abort_events{
      MissionEventType::telemetry_disconnected, MissionEventType::arm_rejected,
      MissionEventType::disarm_rejected,         MissionEventType::takeoff_rejected,
      MissionEventType::position_target_rejected,
      MissionEventType::land_rejected,
      MissionEventType::timeout,
      MissionEventType::stale_telemetry,
      MissionEventType::invalid_telemetry,
      MissionEventType::command_failed,
      MissionEventType::abort_requested,
  };
  for (const auto event : abort_events) {
    drone_lab::FlightStateMachine machine{drone_lab::MissionScenario::square};
    transition = machine.handle(MissionEvent{event, 1, std::nullopt});
    require(transition.accepted && transition.next_state == FlightState::aborted,
            "typed abort event did not enter Aborted");
  }

  drone_lab::FlightStateMachine action_events{drone_lab::MissionScenario::takeoff_only};
  static_cast<void>(action_events.handle(
      MissionEvent{MissionEventType::telemetry_connected, 1, std::nullopt}));
  static_cast<void>(
      action_events.handle(MissionEvent{MissionEventType::vehicle_ready, 2, std::nullopt}));
  static_cast<void>(
      action_events.handle(MissionEvent{MissionEventType::arm_requested, 3, std::nullopt}));
  require(action_events.handle(
              MissionEvent{MissionEventType::arm_accepted, 4, std::uint64_t{1}})
              .accepted,
          "arm acceptance event rejected");
  static_cast<void>(
      action_events.handle(MissionEvent{MissionEventType::armed, 5, std::nullopt}));
  require(action_events.handle(
              MissionEvent{MissionEventType::takeoff_requested, 6, std::uint64_t{2}})
              .accepted,
          "takeoff request event rejected");
  require(action_events.handle(
              MissionEvent{MissionEventType::takeoff_accepted, 7, std::uint64_t{2}})
              .accepted,
          "takeoff acceptance event rejected");
  static_cast<void>(action_events.handle(
      MissionEvent{MissionEventType::takeoff_altitude_reached, 8, std::nullopt}));
  require(action_events.handle(
              MissionEvent{MissionEventType::land_requested, 9, std::uint64_t{3}})
              .accepted,
          "landing request event rejected");
  require(action_events.handle(
              MissionEvent{MissionEventType::land_accepted, 10, std::uint64_t{3}})
              .accepted,
          "landing acceptance event rejected");
  require(action_events.handle(
              MissionEvent{MissionEventType::disarm_requested, 11, std::uint64_t{4}})
              .accepted,
          "landing disarm request event rejected");
  require(action_events.handle(
              MissionEvent{MissionEventType::disarm_accepted, 12, std::uint64_t{4}})
              .accepted,
          "landing disarm acceptance event rejected");

  drone_lab::FlightStateMachine position_events{drone_lab::MissionScenario::single_leg};
  static_cast<void>(position_events.handle(
      MissionEvent{MissionEventType::telemetry_connected, 1, std::nullopt}));
  static_cast<void>(
      position_events.handle(MissionEvent{MissionEventType::arm_requested, 2, std::nullopt}));
  static_cast<void>(
      position_events.handle(MissionEvent{MissionEventType::armed, 3, std::nullopt}));
  static_cast<void>(position_events.handle(
      MissionEvent{MissionEventType::takeoff_altitude_reached, 4, std::nullopt}));
  require(position_events.handle(
              MissionEvent{MissionEventType::position_target_requested, 5, std::uint64_t{3}})
              .accepted,
          "position request event rejected");
  require(position_events.handle(
              MissionEvent{MissionEventType::position_target_accepted, 6, std::uint64_t{3}})
              .accepted,
          "position acceptance event rejected");
}

void target_contract() {
  auto config = fast_config();
  config.leg_length_m = 2.0;
  const auto targets = drone_lab::make_square_targets(config);
  require(targets[0].offset_m.x == 2.0 && targets[0].offset_m.y == 0.0,
          "first corner is not +X");
  require(targets[1].offset_m.x == 2.0 && targets[1].offset_m.y == 2.0,
          "second corner is not +Y");
  require(targets[3].offset_m.x == 0.0 && targets[3].offset_m.y == 0.0,
          "last corner does not return to origin");
  require(drone_lab::valid_position_target(targets[0], config),
          "bounded target rejected");
  config.maximum_position_target_m = 2.0;
  require(!drone_lab::valid_position_target(targets[0], config),
          "excessive target accepted");
}

void nominal_scenarios() {
  struct Expectation {
    drone_lab::MissionScenario scenario;
    std::size_t legs;
    std::uint64_t transitions;
    std::uint64_t move_commands;
  };
  const std::array expectations{
      Expectation{drone_lab::MissionScenario::arm_only, 0, 4, 0},
      Expectation{drone_lab::MissionScenario::takeoff_only, 0, 5, 0},
      Expectation{drone_lab::MissionScenario::single_leg, 1, 6, 1},
      Expectation{drone_lab::MissionScenario::square, 4, 9, 4},
  };
  for (const auto& expectation : expectations) {
    StringRecorder recorder;
    drone_lab::FakeFlightVehicle vehicle;
    const auto result = run(expectation.scenario, recorder, vehicle);
    require(result.final_state == drone_lab::FlightState::complete,
            "nominal scenario did not complete");
    require(result.abort_reason == drone_lab::AbortReason::none, "nominal scenario aborted");
    require(result.transitions == expectation.transitions, "unexpected transition count");
    require(result.metrics.completed_legs == expectation.legs, "wrong completed leg count");
    require(vehicle.command_count(drone_lab::FakeFlightCommand::move_to) ==
                expectation.move_commands,
            "wrong position command count");
    require(result.metrics.final_landed && !result.metrics.final_armed,
            "nominal scenario did not finish landed and disarmed");
    require(!result.stale_command_active, "nominal scenario left stale motion");
    require(recorder.log().find("\"session_id\":\"backyard-flyer\"") != std::string::npos,
            "session id missing from event log");
    require(recorder.log().find("\"transition_accepted\":true") != std::string::npos,
            "accepted transition missing from event log");
    require(recorder.log().find("\"command_sequence\":") != std::string::npos,
            "command sequence missing from event log");
  }
}

void require_abort(drone_lab::FakeFlightFaults faults,
                   drone_lab::MissionScenario scenario,
                   drone_lab::AbortReason expected_reason,
                   const drone_lab::BackyardFlyerConfig* config = nullptr) {
  StringRecorder recorder;
  drone_lab::FakeFlightVehicle vehicle{faults};
  const auto result = run(scenario, recorder, vehicle, config);
  require(result.final_state == drone_lab::FlightState::aborted, "fault did not abort");
  require(result.abort_reason == expected_reason, "fault produced wrong abort reason");
  require(!result.stale_command_active, "abort left stale motion active");
  if (result.metrics.safety_event_count != 1) {
    throw std::runtime_error("abort safety event was not counted for " +
                             std::string{drone_lab::to_string(expected_reason)});
  }
}

void timeout_and_telemetry_failures() {
  require_abort({.start_disconnected = true}, drone_lab::MissionScenario::square,
                drone_lab::AbortReason::timeout);
  require_abort({.disconnect_at_s = 0.05}, drone_lab::MissionScenario::square,
                drone_lab::AbortReason::disconnected);
  require_abort({.disconnect_at_s = 0.8}, drone_lab::MissionScenario::square,
                drone_lab::AbortReason::disconnected);
  require_abort({.stale_telemetry_at_s = 0.2}, drone_lab::MissionScenario::square,
                drone_lab::AbortReason::stale_telemetry);
  require_abort({.invalid_telemetry_at_s = 0.2}, drone_lab::MissionScenario::square,
                drone_lab::AbortReason::invalid_telemetry);
  require_abort({.invalid_attitude_at_s = 0.2}, drone_lab::MissionScenario::square,
                drone_lab::AbortReason::invalid_telemetry);
  require_abort({.not_ready = true}, drone_lab::MissionScenario::square,
                drone_lab::AbortReason::timeout);
  require_abort({.freeze_arm_confirmation = true}, drone_lab::MissionScenario::square,
                drone_lab::AbortReason::timeout);
  require_abort({.freeze_disarm_confirmation = true}, drone_lab::MissionScenario::arm_only,
                drone_lab::AbortReason::timeout);
  require_abort({.freeze_takeoff = true}, drone_lab::MissionScenario::takeoff_only,
                drone_lab::AbortReason::timeout);
  require_abort({.freeze_motion = true}, drone_lab::MissionScenario::single_leg,
                drone_lab::AbortReason::timeout);
  require_abort({.freeze_landing = true}, drone_lab::MissionScenario::takeoff_only,
                drone_lab::AbortReason::timeout);
}

void command_rejections() {
  using drone_lab::FakeFlightCommand;
  for (const auto command :
       {FakeFlightCommand::arm, FakeFlightCommand::disarm, FakeFlightCommand::takeoff,
        FakeFlightCommand::move_to, FakeFlightCommand::land}) {
    const auto scenario = command == FakeFlightCommand::disarm
                              ? drone_lab::MissionScenario::arm_only
                              : drone_lab::MissionScenario::square;
    require_abort({.reject_command = command}, scenario,
                  drone_lab::AbortReason::command_rejected);
  }

  auto config = fast_config(drone_lab::MissionScenario::single_leg);
  config.maximum_position_target_m = 1.1;
  config.takeoff_altitude_m = 1.0;
  config.leg_length_m = 1.0;
  StringRecorder recorder;
  drone_lab::FakeFlightVehicle vehicle;
  const auto result =
      run(drone_lab::MissionScenario::single_leg, recorder, vehicle, &config);
  require(result.final_state == drone_lab::FlightState::aborted,
          "mission did not reject excessive target");
  require(result.abort_reason == drone_lab::AbortReason::command_rejected,
          "excessive target produced wrong reason");
  require(vehicle.command_count(FakeFlightCommand::move_to) == 0,
          "excessive target reached the adapter");
}

void invalid_config_rejected() {
  drone_lab::FakeFlightVehicle vehicle;
  StringRecorder recorder;
  auto config = fast_config();
  config.update_rate_hz = 0.0;
  bool threw = false;
  try {
    drone_lab::BackyardFlyerMission mission{vehicle, recorder, config};
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  require(threw, "invalid configuration was accepted");
}

}  // namespace

int main() {
  try {
    state_machine_contract();
    target_contract();
    nominal_scenarios();
    timeout_and_telemetry_failures();
    command_rejections();
    invalid_config_rejected();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
