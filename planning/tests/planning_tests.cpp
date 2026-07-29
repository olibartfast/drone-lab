#include "drone_lab/planning/planning.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace planning = drone_lab::planning;

namespace {
int failures = 0;

void check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

template <typename Callable>
void check_throws(Callable callable, const char* message) {
  try {
    callable();
    check(false, message);
  } catch (const std::invalid_argument&) {
  }
}

planning::OccupancyGrid open_grid(std::size_t width, std::size_t height) {
  return {width, height, 1.0, {-2.0, -3.0},
          std::vector<planning::Occupancy>(width * height, planning::Occupancy::free), 9};
}
}  // namespace

int main() {
  using enum planning::Occupancy;
  {
    const auto grid = open_grid(4, 3);
    check(grid.world_to_cell({-2.0, -3.0}) == planning::GridIndex{0, 0},
          "minimum corner maps to first cell");
    check(grid.cell_center({0, 0}) == planning::Position2d{-1.5, -2.5},
          "cell centre uses origin and resolution");
    check(grid.world_to_cell({1.999999, -0.000001}) == planning::GridIndex{2, 3},
          "just-inside maximum is accepted");
    check(!grid.world_to_cell({2.0, 0.0}), "exact maximum is excluded");
    check(!grid.world_to_cell({-2.000001, -3.0}), "below origin is excluded");
    check(!grid.world_to_cell({std::numeric_limits<double>::quiet_NaN(), 0.0}),
          "non-finite point is excluded");
  }
  check_throws([] { planning::OccupancyGrid(0, 1, 1.0, {}, {}, 1); },
               "zero grid dimensions rejected");
  check_throws([] { planning::OccupancyGrid(1, 1, 0.0, {}, {free}, 1); },
               "zero grid resolution rejected");
  check_throws([] { planning::OccupancyGrid(1, 1, 1.0, {}, {}, 1); },
               "grid storage mismatch rejected");

  {
    std::vector<planning::Occupancy> cells(25, free);
    cells[12] = occupied;
    const planning::OccupancyGrid source(5, 5, 1.0, {}, cells, 2);
    const auto unchanged = planning::inflate_grid(
        source, 0.0, 0.0, planning::UnknownPolicy::blocked);
    check(planning::blocked_cell_count(unchanged.grid, planning::UnknownPolicy::blocked) == 1,
          "zero envelope preserves occupied count");
    const auto inflated = planning::inflate_grid(
        source, 0.1, 0.0, planning::UnknownPolicy::blocked);
    check(planning::blocked_cell_count(inflated.grid, planning::UnknownPolicy::blocked) == 9,
          "sub-cell envelope conservatively includes touching neighbors");
    check(source.at({2, 1}) == free, "inflation leaves source immutable");
    const auto non_integer = planning::inflate_grid(
        source, 1.1, 0.0, planning::UnknownPolicy::blocked);
    check(planning::blocked_cell_count(non_integer.grid,
                                       planning::UnknownPolicy::blocked) == 21,
          "non-integer envelope uses metric cell-boundary distance");
    cells[0] = unknown;
    const planning::OccupancyGrid with_unknown(5, 5, 1.0, {}, cells, 2);
    check(planning::is_blocked(with_unknown, {0, 0}, planning::UnknownPolicy::blocked) &&
              !planning::is_blocked(with_unknown, {0, 0}, planning::UnknownPolicy::free),
          "unknown-space policy is explicit");
  }

  {
    auto cells = std::vector<planning::Occupancy>(49, free);
    for (std::size_t row = 0; row < 6; ++row) {
      cells[row * 7 + 3] = occupied;
    }
    const planning::OccupancyGrid grid(7, 7, 1.0, {}, cells, 3);
    const planning::GridAStarPlanner grid_planner(grid);
    const planning::GraphAStarPlanner graph_planner(grid);
    const auto grid_result = grid_planner.plan({3, 0}, {3, 6});
    const auto graph_result = graph_planner.plan({3, 0}, {3, 6});
    check(grid_result.status == planning::PlanStatus::success, "grid A* finds detour");
    check(graph_result.status == planning::PlanStatus::success, "graph A* finds detour");
    check(std::abs(grid_result.path_cost_m - graph_result.path_cost_m) < 1e-12,
          "grid and graph optimal costs match");
    check(grid_result.ordered_path.front() == planning::GridIndex{3, 0} &&
              grid_result.ordered_path.back() == planning::GridIndex{3, 6},
          "path includes ordered endpoints");
    const auto repeated = grid_planner.plan({3, 0}, {3, 6});
    check(repeated.ordered_path == grid_result.ordered_path &&
              repeated.expanded_node_count == grid_result.expanded_node_count,
          "grid A* result and metrics are deterministic");
    const auto pruned = planning::prune_path(grid, grid_result.ordered_path);
    check(pruned.rejection_reason == planning::RejectionReason::none,
          "path pruning succeeds");
    check(pruned.path.size() < grid_result.ordered_path.size(),
          "detour pruning removes waypoints");
    check(pruned.path.front() == grid_result.ordered_path.front() &&
              pruned.path.back() == grid_result.ordered_path.back(),
          "pruning preserves endpoints");
    for (std::size_t index = 1; index < pruned.path.size(); ++index) {
      check(planning::has_line_of_sight(grid, pruned.path[index - 1], pruned.path[index]),
            "every pruned segment is collision free");
    }
    check(grid_planner.plan({0, 0}, {0, 0}).ordered_path.size() == 1,
          "start equals goal returns one waypoint");
    check(grid_planner.plan({-1, 0}, {0, 0}).rejection_reason ==
              planning::RejectionReason::start_out_of_bounds,
          "invalid start is typed");
  }

  {
    const planning::OccupancyGrid corner_grid(
        2, 2, 1.0, {}, {free, occupied, occupied, free}, 4);
    const planning::GridAStarPlanner planner(corner_grid);
    check(planner.plan({0, 0}, {1, 1}).rejection_reason ==
              planning::RejectionReason::no_path,
          "diagonal corner cutting is forbidden");
    check(!planning::has_line_of_sight(corner_grid, {0, 0}, {1, 1}),
          "line of sight uses same corner policy");
    check(planning::prune_path(corner_grid, {}).rejection_reason ==
              planning::RejectionReason::path_empty,
          "empty prune input is typed");
  }

  {
    std::vector<planning::Occupancy> voxels(5 * 5 * 4, free);
    voxels[(1 * 5 + 2) * 5 + 2] = occupied;
    const planning::VoxelMap source(5, 5, 4, 1.0, {-1.0, -1.0, 0.0}, voxels, 5);
    check(source.world_to_voxel({-1.0, -1.0, 0.0}) == planning::VoxelIndex{0, 0, 0},
          "voxel minimum corner maps correctly");
    check(!source.world_to_voxel({4.0, 4.0, 4.0}),
          "voxel maximum bound is excluded");
    const auto collision = planning::inflate_voxel_map(
        source, {0.1, 0.1, 0.0, 0.0}, planning::UnknownPolicy::blocked);
    check(collision.at({0, 1, 1}) == occupied &&
              collision.at({2, 3, 3}) == occupied,
          "3D inflation accounts for horizontal and vertical envelope");
    const auto zero_envelope = planning::inflate_voxel_map(
        source, {}, planning::UnknownPolicy::blocked);
    check(zero_envelope.voxels() == source.voxels(),
          "zero 3D envelope preserves known occupancy");

    const planning::PathValidationLimits limits{2.0, 3.0, 10.0, 0.0};
    const std::vector<planning::TimedWaypoint> valid{
        {{-0.5, -0.5, 3.5}, std::chrono::duration<double>(0.0)},
        {{0.5, -0.5, 3.5}, std::chrono::duration<double>(1.0)}};
    check(planning::validate_path(source, collision, valid, limits).status ==
              planning::ValidationStatus::valid,
          "valid timed 3D path passes");
    const std::vector<planning::TimedWaypoint> collision_path{
        {{-0.5, 1.5, 1.5}, std::chrono::duration<double>(0.0)},
        {{3.5, 1.5, 1.5}, std::chrono::duration<double>(4.0)}};
    const auto collision_result =
        planning::validate_path(source, source, collision_path, {}, planning::UnknownPolicy::blocked);
    check(collision_result.status == planning::ValidationStatus::invalid &&
              collision_result.first_invalid_segment == 0,
          "interior segment collision is rejected with segment index");
    const std::vector<planning::TimedWaypoint> fast_climb{
        {{-0.5, -0.5, 0.5}, std::chrono::duration<double>(0.0)},
        {{-0.5, -0.5, 3.5}, std::chrono::duration<double>(1.0)}};
    const auto climb_result = planning::validate_path(
        source, source, fast_climb, planning::PathValidationLimits{2.0, {}, {}, {}});
    check(!climb_result.reasons.empty() &&
              climb_result.reasons.back() == planning::RejectionReason::climb_rate_exceeded,
          "climb-rate rejection is machine readable");
    const std::vector<planning::TimedWaypoint> duplicate{
        {{-0.5, -0.5, 0.5}, std::chrono::duration<double>(0.0)},
        {{-0.5, -0.5, 0.5}, std::chrono::duration<double>(0.0)}};
    const auto duplicate_result = planning::validate_path(source, source, duplicate, {});
    check(duplicate_result.reasons.size() >= 2,
          "duplicate and non-monotonic time failures are both reported");
    const std::vector<planning::TimedWaypoint> out_of_bounds{
        {{-2.0, -0.5, 0.5}, std::chrono::duration<double>(0.0)}};
    check(planning::validate_path(source, source, out_of_bounds, {}).reasons.front() ==
              planning::RejectionReason::waypoint_out_of_bounds,
          "out-of-bounds waypoint rejection is typed");
    const std::vector<planning::TimedWaypoint> long_segment{
        {{-0.5, -0.5, 3.5}, std::chrono::duration<double>(0.0)},
        {{3.5, -0.5, 3.5}, std::chrono::duration<double>(4.0)}};
    const auto limit_result = planning::validate_path(
        source, source, long_segment,
        planning::PathValidationLimits{{}, 3.0, 3.5, {}});
    check(std::find(limit_result.reasons.begin(), limit_result.reasons.end(),
                    planning::RejectionReason::waypoint_spacing_exceeded) !=
              limit_result.reasons.end() &&
              std::find(limit_result.reasons.begin(), limit_result.reasons.end(),
                        planning::RejectionReason::path_length_exceeded) !=
                  limit_result.reasons.end(),
          "spacing and total-length violations are both reported");
    const auto invalid_limits = planning::validate_path(
        source, source, valid, planning::PathValidationLimits{-1.0, {}, {}, {}});
    check(invalid_limits.reasons.front() ==
              planning::RejectionReason::invalid_configuration,
          "negative validation limit is rejected");
  }

  check(drone_lab::planning::to_string(planning::RejectionReason::no_path) == "no_path",
        "stable rejection strings");
  if (failures == 0) {
    std::cout << "{\"schema_version\":1,\"suite\":\"planning\",\"status\":\"passed\"}\n";
  }
  return failures == 0 ? 0 : 1;
}
