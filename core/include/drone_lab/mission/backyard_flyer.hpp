#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "drone_lab/platform/interfaces.hpp"

namespace drone_lab {

enum class MissionScenario {
  arm_only,
  takeoff_only,
  single_leg,
  square,
};

enum class FlightState {
  disconnected,
  ready,
  arming,
  disarming,
  taking_off,
  flying_leg_1,
  flying_leg_2,
  flying_leg_3,
  flying_leg_4,
  landing,
  complete,
  aborted,
};

inline constexpr std::size_t kFlightStateCount = 12;
inline constexpr std::size_t kMaximumLegs = 4;

enum class MissionEventType {
  command_requested,
  telemetry_connected,
  telemetry_disconnected,
  vehicle_ready,
  arm_requested,
  arm_accepted,
  arm_rejected,
  armed,
  disarm_requested,
  disarm_accepted,
  disarm_rejected,
  disarmed,
  takeoff_requested,
  takeoff_accepted,
  takeoff_rejected,
  takeoff_altitude_reached,
  position_target_requested,
  position_target_accepted,
  position_target_rejected,
  leg_target_reached,
  land_requested,
  land_accepted,
  land_rejected,
  landed,
  timeout,
  stale_telemetry,
  invalid_telemetry,
  command_failed,
  abort_requested,
};

enum class TransitionReason {
  accepted,
  abort,
  invalid_source_state,
  duplicate_event,
  terminal_state,
};

enum class MissionActionType {
  none,
  arm,
  disarm,
  takeoff,
  position_target,
  land,
  stop,
};

enum class AbortReason {
  none,
  disconnected,
  timeout,
  command_rejected,
  invalid_telemetry,
  stale_telemetry,
};

struct MissionEvent {
  MissionEventType type{MissionEventType::abort_requested};
  TimestampNs timestamp_ns{0};
  std::optional<std::uint64_t> command_sequence{};
};

struct MissionActionProposal {
  MissionActionType type{MissionActionType::none};
  std::optional<std::size_t> leg_index{};
};

struct TransitionResult {
  FlightState previous_state{FlightState::disconnected};
  FlightState next_state{FlightState::disconnected};
  MissionEventType event{MissionEventType::abort_requested};
  TransitionReason reason{TransitionReason::invalid_source_state};
  TimestampNs timestamp_ns{0};
  std::optional<std::uint64_t> command_sequence{};
  bool accepted{false};
};

class FlightStateMachine {
 public:
  explicit FlightStateMachine(MissionScenario scenario);

  [[nodiscard]] FlightState state() const noexcept;
  [[nodiscard]] bool terminal() const noexcept;
  [[nodiscard]] MissionActionProposal entry_action() const noexcept;
  [[nodiscard]] TransitionResult handle(const MissionEvent& event) noexcept;

 private:
  MissionScenario scenario_;
  FlightState state_{FlightState::disconnected};
  std::optional<MissionEventType> last_event_{};
  FlightState last_event_state_{FlightState::disconnected};
};

struct RelativePositionTarget {
  Vector3 offset_m{};
  CoordinateFrame frame{CoordinateFrame::world_enu};
};

struct BackyardFlyerConfig {
  std::uint32_t version{1};
  MissionScenario scenario{MissionScenario::square};
  std::string session_id{"backyard-flyer"};
  double update_rate_hz{20.0};
  double takeoff_altitude_m{2.0};
  double leg_length_m{4.0};
  double horizontal_speed_mps{1.0};
  double vertical_speed_mps{0.8};
  double position_tolerance_m{0.15};
  double altitude_tolerance_m{0.10};
  double confirmation_time_s{1.0};
  double telemetry_max_age_s{0.5};
  double connection_timeout_s{5.0};
  double ready_timeout_s{5.0};
  double arm_timeout_s{10.0};
  double takeoff_timeout_s{20.0};
  double leg_timeout_s{15.0};
  double landing_timeout_s{20.0};
  double mission_timeout_s{90.0};
  double maximum_horizontal_speed_mps{2.0};
  double maximum_vertical_speed_mps{1.0};
  double maximum_position_target_m{10.0};
};

struct LegMetrics {
  Vector3 start_position_m{};
  Vector3 target_position_m{};
  double duration_s{0.0};
  double endpoint_error_m{0.0};
  double maximum_cross_track_error_m{0.0};
  double maximum_altitude_deviation_m{0.0};
  double maximum_speed_mps{0.0};
  bool completed{false};
};

struct BackyardFlyerMetrics {
  double total_duration_s{0.0};
  std::array<double, kFlightStateCount> state_durations_s{};
  std::uint64_t command_count{0};
  std::uint64_t command_rejection_count{0};
  std::uint64_t timeout_count{0};
  std::uint64_t connection_loss_count{0};
  std::uint64_t safety_event_count{0};
  double maximum_horizontal_speed_mps{0.0};
  double maximum_vertical_speed_mps{0.0};
  double maximum_altitude_error_m{0.0};
  double connection_latency_s{0.0};
  double readiness_latency_s{0.0};
  double arm_request_latency_s{0.0};
  double arm_confirmation_latency_s{0.0};
  double takeoff_request_time_s{0.0};
  double time_to_altitude_tolerance_s{0.0};
  double final_takeoff_altitude_error_m{0.0};
  std::uint64_t vertical_speed_limit_violations{0};
  std::array<LegMetrics, kMaximumLegs> legs{};
  std::size_t completed_legs{0};
  double total_path_time_s{0.0};
  double final_origin_error_m{0.0};
  bool frame_sign_verified{false};
  double landing_request_time_s{0.0};
  double touchdown_time_s{0.0};
  double landing_duration_s{0.0};
  bool final_landed{false};
  bool final_armed{false};
};

struct BackyardFlyerResult {
  MissionScenario scenario{MissionScenario::square};
  FlightState final_state{FlightState::disconnected};
  AbortReason abort_reason{AbortReason::none};
  std::uint64_t ticks{0};
  std::uint64_t transitions{0};
  std::uint64_t commands_accepted{0};
  std::uint64_t commands_rejected{0};
  double endpoint_error_m{0.0};
  bool stale_command_active{false};
  BackyardFlyerMetrics metrics{};
};

class FlightVehicle : public TelemetrySource {
 public:
  ~FlightVehicle() override = default;
  virtual CommandResult arm() = 0;
  virtual CommandResult disarm() = 0;
  virtual CommandResult takeoff(double altitude_m) = 0;
  virtual CommandResult move_to(const Vector3& position_m, double speed_mps) = 0;
  virtual CommandResult land() = 0;
  virtual CommandResult stop_motion() = 0;
  virtual void advance(double dt_s) = 0;
};

class BackyardFlyerMission {
 public:
  BackyardFlyerMission(FlightVehicle& vehicle, Recorder& recorder,
                       BackyardFlyerConfig config = {});
  [[nodiscard]] BackyardFlyerResult run();

 private:
  FlightVehicle& vehicle_;
  Recorder& recorder_;
  BackyardFlyerConfig config_;
};

[[nodiscard]] std::array<RelativePositionTarget, kMaximumLegs> make_square_targets(
    const BackyardFlyerConfig& config);
[[nodiscard]] bool valid_position_target(const RelativePositionTarget& target,
                                         const BackyardFlyerConfig& config) noexcept;

[[nodiscard]] std::string_view to_string(MissionScenario scenario) noexcept;
[[nodiscard]] std::string_view to_string(FlightState state) noexcept;
[[nodiscard]] std::string_view to_string(MissionEventType event) noexcept;
[[nodiscard]] std::string_view to_string(TransitionReason reason) noexcept;
[[nodiscard]] std::string_view to_string(MissionActionType action) noexcept;
[[nodiscard]] std::string_view to_string(AbortReason reason) noexcept;

}  // namespace drone_lab
