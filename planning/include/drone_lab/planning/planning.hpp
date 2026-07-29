#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace drone_lab::planning {

struct Position2d {
  double x_m{0.0};
  double y_m{0.0};
  friend bool operator==(const Position2d&, const Position2d&) = default;
};

struct Position3d {
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
  friend bool operator==(const Position3d&, const Position3d&) = default;
};

struct GridIndex {
  std::int32_t row{0};
  std::int32_t column{0};
  friend bool operator==(const GridIndex&, const GridIndex&) = default;
};

struct VoxelIndex {
  std::int32_t layer{0};
  std::int32_t row{0};
  std::int32_t column{0};
  friend bool operator==(const VoxelIndex&, const VoxelIndex&) = default;
};

enum class Occupancy : std::uint8_t { free, occupied, unknown };
enum class UnknownPolicy : std::uint8_t { blocked, free };
enum class PlannerKind : std::uint8_t { grid, graph };
enum class PlanStatus : std::uint8_t { success, rejected };
enum class ValidationStatus : std::uint8_t { valid, invalid };

enum class RejectionReason : std::uint8_t {
  none,
  invalid_configuration,
  invalid_map_dimensions,
  invalid_resolution,
  non_finite_value,
  unsupported_fixture_version,
  malformed_fixture,
  start_out_of_bounds,
  goal_out_of_bounds,
  start_blocked,
  goal_blocked,
  no_path,
  path_empty,
  waypoint_out_of_bounds,
  waypoint_in_collision,
  segment_in_collision,
  insufficient_clearance,
  duplicate_waypoint,
  non_monotonic_time,
  climb_rate_exceeded,
  waypoint_spacing_exceeded,
  path_length_exceeded,
  output_write_failed,
  internal_invariant_failed,
};

[[nodiscard]] std::string_view to_string(UnknownPolicy value) noexcept;
[[nodiscard]] std::string_view to_string(PlannerKind value) noexcept;
[[nodiscard]] std::string_view to_string(PlanStatus value) noexcept;
[[nodiscard]] std::string_view to_string(ValidationStatus value) noexcept;
[[nodiscard]] std::string_view to_string(RejectionReason value) noexcept;

class OccupancyGrid {
 public:
  OccupancyGrid(std::size_t width, std::size_t height, double resolution_m,
                Position2d origin_m, std::vector<Occupancy> cells,
                std::uint64_t revision = 1);

  [[nodiscard]] std::size_t width() const noexcept;
  [[nodiscard]] std::size_t height() const noexcept;
  [[nodiscard]] double resolution_m() const noexcept;
  [[nodiscard]] Position2d origin_m() const noexcept;
  [[nodiscard]] std::uint64_t revision() const noexcept;
  [[nodiscard]] bool in_bounds(GridIndex index) const noexcept;
  [[nodiscard]] std::optional<Occupancy> at(GridIndex index) const noexcept;
  [[nodiscard]] std::optional<GridIndex> world_to_cell(Position2d point_m) const noexcept;
  [[nodiscard]] std::optional<Position2d> cell_center(GridIndex index) const noexcept;
  [[nodiscard]] const std::vector<Occupancy>& cells() const noexcept;

 private:
  std::size_t width_{};
  std::size_t height_{};
  double resolution_m_{};
  Position2d origin_m_{};
  std::vector<Occupancy> cells_{};
  std::uint64_t revision_{};
};

struct InflatedGrid {
  OccupancyGrid grid;
  double vehicle_radius_m{0.0};
  double clearance_m{0.0};
  UnknownPolicy unknown_policy{UnknownPolicy::blocked};
};

[[nodiscard]] InflatedGrid inflate_grid(const OccupancyGrid& source,
                                        double vehicle_radius_m,
                                        double clearance_m,
                                        UnknownPolicy unknown_policy);
[[nodiscard]] bool is_blocked(const OccupancyGrid& grid, GridIndex index,
                              UnknownPolicy unknown_policy) noexcept;
[[nodiscard]] std::size_t blocked_cell_count(const OccupancyGrid& grid,
                                             UnknownPolicy unknown_policy) noexcept;

struct Plan2dResult {
  PlanStatus status{PlanStatus::rejected};
  RejectionReason rejection_reason{RejectionReason::invalid_configuration};
  GridIndex start{};
  GridIndex goal{};
  std::vector<GridIndex> ordered_path{};
  double path_cost_m{0.0};
  std::size_t expanded_node_count{0};
  std::size_t generated_node_count{0};
  PlannerKind planner_kind{PlannerKind::grid};
  std::uint64_t map_revision{0};
};

class Planner2d {
 public:
  virtual ~Planner2d() = default;
  [[nodiscard]] virtual Plan2dResult plan(GridIndex start, GridIndex goal) const = 0;
};

class GridAStarPlanner final : public Planner2d {
 public:
  GridAStarPlanner(const OccupancyGrid& grid,
                   UnknownPolicy unknown_policy = UnknownPolicy::blocked);
  [[nodiscard]] Plan2dResult plan(GridIndex start, GridIndex goal) const override;

 private:
  const OccupancyGrid& grid_;
  UnknownPolicy unknown_policy_;
};

struct GraphEdge {
  std::size_t destination{0};
  double weight_m{0.0};
};

struct GraphNode {
  std::size_t id{0};
  GridIndex cell{};
  Position2d position_m{};
  std::vector<GraphEdge> edges{};
};

class FreeSpaceGraph {
 public:
  [[nodiscard]] static FreeSpaceGraph from_grid(
      const OccupancyGrid& grid, UnknownPolicy unknown_policy = UnknownPolicy::blocked);
  [[nodiscard]] const std::vector<GraphNode>& nodes() const noexcept;
  [[nodiscard]] std::optional<std::size_t> node_for(GridIndex cell) const noexcept;

 private:
  std::size_t grid_width_{0};
  std::vector<GraphNode> nodes_{};
  std::vector<std::optional<std::size_t>> cell_to_node_{};
};

class GraphAStarPlanner final : public Planner2d {
 public:
  GraphAStarPlanner(const OccupancyGrid& grid,
                    UnknownPolicy unknown_policy = UnknownPolicy::blocked);
  [[nodiscard]] Plan2dResult plan(GridIndex start, GridIndex goal) const override;

 private:
  const OccupancyGrid& grid_;
  FreeSpaceGraph graph_;
};

[[nodiscard]] bool has_line_of_sight(
    const OccupancyGrid& grid, GridIndex from, GridIndex to,
    UnknownPolicy unknown_policy = UnknownPolicy::blocked) noexcept;

struct PruneResult {
  RejectionReason rejection_reason{RejectionReason::none};
  std::vector<GridIndex> path{};
};

[[nodiscard]] PruneResult prune_path(
    const OccupancyGrid& grid, std::span<const GridIndex> path,
    UnknownPolicy unknown_policy = UnknownPolicy::blocked);
[[nodiscard]] double path_length_m(const OccupancyGrid& grid,
                                    std::span<const GridIndex> path) noexcept;

class VoxelMap {
 public:
  VoxelMap(std::size_t width, std::size_t depth, std::size_t height,
           double resolution_m, Position3d origin_m,
           std::vector<Occupancy> voxels, std::uint64_t revision = 1);

  [[nodiscard]] std::size_t width() const noexcept;
  [[nodiscard]] std::size_t depth() const noexcept;
  [[nodiscard]] std::size_t height() const noexcept;
  [[nodiscard]] double resolution_m() const noexcept;
  [[nodiscard]] Position3d origin_m() const noexcept;
  [[nodiscard]] std::uint64_t revision() const noexcept;
  [[nodiscard]] bool in_bounds(VoxelIndex index) const noexcept;
  [[nodiscard]] std::optional<Occupancy> at(VoxelIndex index) const noexcept;
  [[nodiscard]] std::optional<VoxelIndex> world_to_voxel(Position3d point_m) const noexcept;
  [[nodiscard]] std::optional<Position3d> voxel_center(VoxelIndex index) const noexcept;
  [[nodiscard]] const std::vector<Occupancy>& voxels() const noexcept;

 private:
  std::size_t width_{};
  std::size_t depth_{};
  std::size_t height_{};
  double resolution_m_{};
  Position3d origin_m_{};
  std::vector<Occupancy> voxels_{};
  std::uint64_t revision_{};
};

struct VehicleEnvelope3d {
  double vehicle_radius_m{0.0};
  double vehicle_half_height_m{0.0};
  double horizontal_clearance_m{0.0};
  double vertical_clearance_m{0.0};
};

[[nodiscard]] VoxelMap inflate_voxel_map(const VoxelMap& source,
                                         VehicleEnvelope3d envelope,
                                         UnknownPolicy unknown_policy);

struct TimedWaypoint {
  Position3d position_m{};
  std::chrono::duration<double> time_from_start{};
};

struct PathValidationLimits {
  std::optional<double> maximum_climb_rate_mps{};
  std::optional<double> maximum_waypoint_spacing_m{};
  std::optional<double> maximum_path_length_m{};
  std::optional<double> minimum_clearance_m{};
};

struct PathValidationResult {
  ValidationStatus status{ValidationStatus::invalid};
  std::vector<RejectionReason> reasons{};
  std::optional<std::size_t> first_invalid_waypoint{};
  std::optional<std::size_t> first_invalid_segment{};
  std::optional<VoxelIndex> first_colliding_voxel{};
  double maximum_observed_spacing_m{0.0};
  double total_path_length_m{0.0};
  double minimum_observed_clearance_m{0.0};
};

[[nodiscard]] PathValidationResult validate_path(
    const VoxelMap& source, const VoxelMap& collision_map,
    std::span<const TimedWaypoint> path, const PathValidationLimits& limits,
    UnknownPolicy unknown_policy = UnknownPolicy::blocked);

}  // namespace drone_lab::planning
