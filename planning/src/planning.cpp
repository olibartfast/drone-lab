#include "drone_lab/planning/planning.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace drone_lab::planning {
namespace {

[[nodiscard]] bool finite(Position2d value) noexcept {
  return std::isfinite(value.x_m) && std::isfinite(value.y_m);
}

[[nodiscard]] bool finite(Position3d value) noexcept {
  return std::isfinite(value.x_m) && std::isfinite(value.y_m) && std::isfinite(value.z_m);
}

[[nodiscard]] bool valid_occupancy(Occupancy value) noexcept {
  return value == Occupancy::free || value == Occupancy::occupied || value == Occupancy::unknown;
}

[[nodiscard]] std::size_t checked_product(std::size_t a, std::size_t b) {
  if (a == 0 || b == 0 || a > std::numeric_limits<std::size_t>::max() / b) {
    throw std::invalid_argument("invalid map dimensions");
  }
  return a * b;
}

[[nodiscard]] std::size_t linear(GridIndex index, std::size_t width) noexcept {
  return static_cast<std::size_t>(index.row) * width +
         static_cast<std::size_t>(index.column);
}

[[nodiscard]] std::size_t linear(VoxelIndex index, std::size_t width,
                                 std::size_t depth) noexcept {
  return (static_cast<std::size_t>(index.layer) * depth +
          static_cast<std::size_t>(index.row)) * width +
         static_cast<std::size_t>(index.column);
}

constexpr std::array<GridIndex, 8> kNeighborOffsets{{
    {-1, 0}, {0, 1}, {1, 0}, {0, -1},
    {-1, 1}, {1, 1}, {1, -1}, {-1, -1},
}};

[[nodiscard]] bool legal_move(const OccupancyGrid& grid, GridIndex from,
                              GridIndex to, UnknownPolicy policy) noexcept {
  if (!grid.in_bounds(to) || is_blocked(grid, to, policy)) {
    return false;
  }
  const auto dr = to.row - from.row;
  const auto dc = to.column - from.column;
  if (dr != 0 && dc != 0) {
    if (is_blocked(grid, {from.row + dr, from.column}, policy) ||
        is_blocked(grid, {from.row, from.column + dc}, policy)) {
      return false;
    }
  }
  return true;
}

[[nodiscard]] double octile(GridIndex from, GridIndex to, double resolution_m) noexcept {
  const auto dx = std::abs(from.column - to.column);
  const auto dy = std::abs(from.row - to.row);
  const auto diagonal = std::min(dx, dy);
  const auto straight = std::max(dx, dy) - diagonal;
  return resolution_m *
         (static_cast<double>(straight) + std::sqrt(2.0) * static_cast<double>(diagonal));
}

struct OpenEntry {
  double f{};
  double h{};
  double g{};
  std::size_t stable_index{};
};

struct OpenLater {
  [[nodiscard]] bool operator()(const OpenEntry& left, const OpenEntry& right) const noexcept {
    return std::tie(left.f, left.h, left.stable_index) >
           std::tie(right.f, right.h, right.stable_index);
  }
};

[[nodiscard]] Plan2dResult reject(const OccupancyGrid& grid, PlannerKind kind,
                                  GridIndex start, GridIndex goal,
                                  RejectionReason reason) {
  Plan2dResult result;
  result.start = start;
  result.goal = goal;
  result.planner_kind = kind;
  result.map_revision = grid.revision();
  result.rejection_reason = reason;
  return result;
}

[[nodiscard]] double distance(Position3d a, Position3d b) noexcept {
  return std::hypot(std::hypot(a.x_m - b.x_m, a.y_m - b.y_m), a.z_m - b.z_m);
}

void add_reason(PathValidationResult& result, RejectionReason reason) {
  if (std::find(result.reasons.begin(), result.reasons.end(), reason) == result.reasons.end()) {
    result.reasons.push_back(reason);
  }
}

[[nodiscard]] bool policy_blocked(Occupancy value, UnknownPolicy policy) noexcept {
  return value == Occupancy::occupied ||
         (value == Occupancy::unknown && policy == UnknownPolicy::blocked);
}

}  // namespace

std::string_view to_string(UnknownPolicy value) noexcept {
  return value == UnknownPolicy::blocked ? "blocked" : "free";
}

std::string_view to_string(PlannerKind value) noexcept {
  return value == PlannerKind::grid ? "grid" : "graph";
}

std::string_view to_string(PlanStatus value) noexcept {
  return value == PlanStatus::success ? "success" : "rejected";
}

std::string_view to_string(ValidationStatus value) noexcept {
  return value == ValidationStatus::valid ? "valid" : "invalid";
}

std::string_view to_string(RejectionReason value) noexcept {
  constexpr std::array<std::string_view, 27> names{{
      "none", "invalid_configuration", "invalid_map_dimensions", "invalid_resolution",
      "non_finite_value", "unsupported_fixture_version", "malformed_fixture",
      "start_out_of_bounds", "goal_out_of_bounds", "start_blocked", "goal_blocked",
      "no_path", "path_empty", "waypoint_out_of_bounds", "waypoint_in_collision",
      "segment_in_collision", "insufficient_clearance", "duplicate_waypoint",
      "non_monotonic_time", "climb_rate_exceeded", "waypoint_spacing_exceeded",
      "path_length_exceeded", "output_write_failed", "internal_invariant_failed",
      "invalid_configuration", "invalid_configuration", "invalid_configuration",
  }};
  const auto index = static_cast<std::size_t>(value);
  return index < 24 ? names[index] : "invalid_configuration";
}

OccupancyGrid::OccupancyGrid(std::size_t width, std::size_t height, double resolution_m,
                             Position2d origin_m, std::vector<Occupancy> cells,
                             std::uint64_t revision)
    : width_(width), height_(height), resolution_m_(resolution_m), origin_m_(origin_m),
      cells_(std::move(cells)), revision_(revision) {
  const auto expected = checked_product(width_, height_);
  if (!(resolution_m_ > 0.0) || !std::isfinite(resolution_m_)) {
    throw std::invalid_argument("invalid resolution");
  }
  if (!finite(origin_m_) || cells_.size() != expected ||
      !std::all_of(cells_.begin(), cells_.end(), valid_occupancy)) {
    throw std::invalid_argument("invalid occupancy grid");
  }
}

std::size_t OccupancyGrid::width() const noexcept { return width_; }
std::size_t OccupancyGrid::height() const noexcept { return height_; }
double OccupancyGrid::resolution_m() const noexcept { return resolution_m_; }
Position2d OccupancyGrid::origin_m() const noexcept { return origin_m_; }
std::uint64_t OccupancyGrid::revision() const noexcept { return revision_; }
const std::vector<Occupancy>& OccupancyGrid::cells() const noexcept { return cells_; }

bool OccupancyGrid::in_bounds(GridIndex index) const noexcept {
  return index.row >= 0 && index.column >= 0 &&
         static_cast<std::size_t>(index.row) < height_ &&
         static_cast<std::size_t>(index.column) < width_;
}

std::optional<Occupancy> OccupancyGrid::at(GridIndex index) const noexcept {
  if (!in_bounds(index)) {
    return std::nullopt;
  }
  return cells_[linear(index, width_)];
}

std::optional<GridIndex> OccupancyGrid::world_to_cell(Position2d point_m) const noexcept {
  if (!finite(point_m)) {
    return std::nullopt;
  }
  const auto column = std::floor((point_m.x_m - origin_m_.x_m) / resolution_m_);
  const auto row = std::floor((point_m.y_m - origin_m_.y_m) / resolution_m_);
  if (column < 0.0 || row < 0.0 ||
      column >= static_cast<double>(width_) || row >= static_cast<double>(height_)) {
    return std::nullopt;
  }
  return GridIndex{static_cast<std::int32_t>(row), static_cast<std::int32_t>(column)};
}

std::optional<Position2d> OccupancyGrid::cell_center(GridIndex index) const noexcept {
  if (!in_bounds(index)) {
    return std::nullopt;
  }
  return Position2d{
      origin_m_.x_m + (static_cast<double>(index.column) + 0.5) * resolution_m_,
      origin_m_.y_m + (static_cast<double>(index.row) + 0.5) * resolution_m_};
}

bool is_blocked(const OccupancyGrid& grid, GridIndex index, UnknownPolicy policy) noexcept {
  const auto value = grid.at(index);
  return !value || policy_blocked(*value, policy);
}

std::size_t blocked_cell_count(const OccupancyGrid& grid, UnknownPolicy policy) noexcept {
  return static_cast<std::size_t>(std::count_if(
      grid.cells().begin(), grid.cells().end(),
      [policy](Occupancy value) { return policy_blocked(value, policy); }));
}

InflatedGrid inflate_grid(const OccupancyGrid& source, double vehicle_radius_m,
                          double clearance_m, UnknownPolicy policy) {
  if (!std::isfinite(vehicle_radius_m) || !std::isfinite(clearance_m) ||
      vehicle_radius_m < 0.0 || clearance_m < 0.0) {
    throw std::invalid_argument("invalid 2D vehicle envelope");
  }
  const auto envelope = vehicle_radius_m + clearance_m;
  auto cells = source.cells();
  if (envelope > 0.0) {
    for (std::size_t row = 0; row < source.height(); ++row) {
      for (std::size_t column = 0; column < source.width(); ++column) {
        const GridIndex target{static_cast<std::int32_t>(row),
                               static_cast<std::int32_t>(column)};
        bool blocked = false;
        for (std::size_t obstacle_row = 0; obstacle_row < source.height() && !blocked;
             ++obstacle_row) {
          for (std::size_t obstacle_column = 0; obstacle_column < source.width();
               ++obstacle_column) {
            const GridIndex obstacle{static_cast<std::int32_t>(obstacle_row),
                                     static_cast<std::int32_t>(obstacle_column)};
            if (!is_blocked(source, obstacle, policy)) {
              continue;
            }
            const auto column_delta = std::abs(target.column - obstacle.column);
            const auto row_delta = std::abs(target.row - obstacle.row);
            const auto dx = static_cast<double>(std::max(0, column_delta - 1)) *
                            source.resolution_m();
            const auto dy = static_cast<double>(std::max(0, row_delta - 1)) *
                            source.resolution_m();
            if (std::hypot(dx, dy) <= envelope) {
              blocked = true;
              break;
            }
          }
        }
        if (blocked) {
          cells[linear(target, source.width())] = Occupancy::occupied;
        }
      }
    }
  } else if (policy == UnknownPolicy::blocked) {
    std::replace(cells.begin(), cells.end(), Occupancy::unknown, Occupancy::occupied);
  }
  return {OccupancyGrid(source.width(), source.height(), source.resolution_m(),
                        source.origin_m(), std::move(cells), source.revision()),
          vehicle_radius_m, clearance_m, policy};
}

GridAStarPlanner::GridAStarPlanner(const OccupancyGrid& grid, UnknownPolicy policy)
    : grid_(grid), unknown_policy_(policy) {}

Plan2dResult GridAStarPlanner::plan(GridIndex start, GridIndex goal) const {
  if (!grid_.in_bounds(start)) {
    return reject(grid_, PlannerKind::grid, start, goal, RejectionReason::start_out_of_bounds);
  }
  if (!grid_.in_bounds(goal)) {
    return reject(grid_, PlannerKind::grid, start, goal, RejectionReason::goal_out_of_bounds);
  }
  if (is_blocked(grid_, start, unknown_policy_)) {
    return reject(grid_, PlannerKind::grid, start, goal, RejectionReason::start_blocked);
  }
  if (is_blocked(grid_, goal, unknown_policy_)) {
    return reject(grid_, PlannerKind::grid, start, goal, RejectionReason::goal_blocked);
  }
  const auto cell_count = checked_product(grid_.width(), grid_.height());
  const auto infinity = std::numeric_limits<double>::infinity();
  std::vector<double> costs(cell_count, infinity);
  std::vector<std::optional<std::size_t>> predecessor(cell_count);
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenLater> open;
  const auto start_index = linear(start, grid_.width());
  const auto goal_index = linear(goal, grid_.width());
  costs[start_index] = 0.0;
  const auto start_h = octile(start, goal, grid_.resolution_m());
  open.push({start_h, start_h, 0.0, start_index});
  std::size_t generated = 1;
  std::size_t expanded = 0;

  while (!open.empty()) {
    const auto current = open.top();
    open.pop();
    if (current.g != costs[current.stable_index]) {
      continue;
    }
    ++expanded;
    if (current.stable_index == goal_index) {
      break;
    }
    const GridIndex cell{
        static_cast<std::int32_t>(current.stable_index / grid_.width()),
        static_cast<std::int32_t>(current.stable_index % grid_.width())};
    for (const auto offset : kNeighborOffsets) {
      const GridIndex next{cell.row + offset.row, cell.column + offset.column};
      if (!legal_move(grid_, cell, next, unknown_policy_)) {
        continue;
      }
      const auto next_index = linear(next, grid_.width());
      const auto edge = (offset.row != 0 && offset.column != 0)
                            ? std::sqrt(2.0) * grid_.resolution_m()
                            : grid_.resolution_m();
      const auto candidate = current.g + edge;
      if (candidate < costs[next_index]) {
        if (!std::isfinite(costs[next_index])) {
          ++generated;
        }
        costs[next_index] = candidate;
        predecessor[next_index] = current.stable_index;
        const auto h = octile(next, goal, grid_.resolution_m());
        open.push({candidate + h, h, candidate, next_index});
      }
    }
  }

  if (!std::isfinite(costs[goal_index])) {
    auto result = reject(grid_, PlannerKind::grid, start, goal, RejectionReason::no_path);
    result.expanded_node_count = expanded;
    result.generated_node_count = generated;
    return result;
  }

  std::vector<GridIndex> reversed;
  auto cursor = goal_index;
  for (std::size_t count = 0; count <= cell_count; ++count) {
    reversed.push_back({
        static_cast<std::int32_t>(cursor / grid_.width()),
        static_cast<std::int32_t>(cursor % grid_.width())});
    if (cursor == start_index) {
      break;
    }
    if (!predecessor[cursor]) {
      return reject(grid_, PlannerKind::grid, start, goal,
                    RejectionReason::internal_invariant_failed);
    }
    cursor = *predecessor[cursor];
  }
  if (reversed.back() != start) {
    return reject(grid_, PlannerKind::grid, start, goal,
                  RejectionReason::internal_invariant_failed);
  }
  std::reverse(reversed.begin(), reversed.end());
  Plan2dResult result;
  result.status = PlanStatus::success;
  result.rejection_reason = RejectionReason::none;
  result.start = start;
  result.goal = goal;
  result.ordered_path = std::move(reversed);
  result.path_cost_m = costs[goal_index];
  result.expanded_node_count = expanded;
  result.generated_node_count = generated;
  result.planner_kind = PlannerKind::grid;
  result.map_revision = grid_.revision();
  return result;
}

FreeSpaceGraph FreeSpaceGraph::from_grid(const OccupancyGrid& grid, UnknownPolicy policy) {
  FreeSpaceGraph graph;
  graph.grid_width_ = grid.width();
  graph.cell_to_node_.resize(checked_product(grid.width(), grid.height()));
  for (std::size_t row = 0; row < grid.height(); ++row) {
    for (std::size_t column = 0; column < grid.width(); ++column) {
      const GridIndex cell{static_cast<std::int32_t>(row), static_cast<std::int32_t>(column)};
      if (is_blocked(grid, cell, policy)) {
        continue;
      }
      const auto id = graph.nodes_.size();
      graph.cell_to_node_[linear(cell, grid.width())] = id;
      graph.nodes_.push_back({id, cell, *grid.cell_center(cell), {}});
    }
  }
  for (auto& node : graph.nodes_) {
    for (const auto offset : kNeighborOffsets) {
      const GridIndex next{node.cell.row + offset.row, node.cell.column + offset.column};
      if (!legal_move(grid, node.cell, next, policy)) {
        continue;
      }
      const auto destination = graph.node_for(next);
      if (destination) {
        const auto weight = (offset.row != 0 && offset.column != 0)
                                ? std::sqrt(2.0) * grid.resolution_m()
                                : grid.resolution_m();
        node.edges.push_back({*destination, weight});
      }
    }
    std::sort(node.edges.begin(), node.edges.end(),
              [](const GraphEdge& a, const GraphEdge& b) {
                return a.destination < b.destination;
              });
  }
  return graph;
}

const std::vector<GraphNode>& FreeSpaceGraph::nodes() const noexcept { return nodes_; }

std::optional<std::size_t> FreeSpaceGraph::node_for(GridIndex cell) const noexcept {
  if (cell.row < 0 || cell.column < 0 ||
      static_cast<std::size_t>(cell.column) >= grid_width_) {
    return std::nullopt;
  }
  const auto index = static_cast<std::size_t>(cell.row) * grid_width_ +
                     static_cast<std::size_t>(cell.column);
  if (index >= cell_to_node_.size()) {
    return std::nullopt;
  }
  return cell_to_node_[index];
}

GraphAStarPlanner::GraphAStarPlanner(const OccupancyGrid& grid, UnknownPolicy policy)
    : grid_(grid), graph_(FreeSpaceGraph::from_grid(grid, policy)) {}

Plan2dResult GraphAStarPlanner::plan(GridIndex start, GridIndex goal) const {
  if (!grid_.in_bounds(start)) {
    return reject(grid_, PlannerKind::graph, start, goal, RejectionReason::start_out_of_bounds);
  }
  if (!grid_.in_bounds(goal)) {
    return reject(grid_, PlannerKind::graph, start, goal, RejectionReason::goal_out_of_bounds);
  }
  const auto start_node = graph_.node_for(start);
  if (!start_node) {
    return reject(grid_, PlannerKind::graph, start, goal, RejectionReason::start_blocked);
  }
  const auto goal_node = graph_.node_for(goal);
  if (!goal_node) {
    return reject(grid_, PlannerKind::graph, start, goal, RejectionReason::goal_blocked);
  }
  const auto infinity = std::numeric_limits<double>::infinity();
  std::vector<double> costs(graph_.nodes().size(), infinity);
  std::vector<std::optional<std::size_t>> predecessor(graph_.nodes().size());
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenLater> open;
  costs[*start_node] = 0.0;
  const auto h0 = octile(start, goal, grid_.resolution_m());
  open.push({h0, h0, 0.0, *start_node});
  std::size_t generated = 1;
  std::size_t expanded = 0;
  while (!open.empty()) {
    const auto current = open.top();
    open.pop();
    if (current.g != costs[current.stable_index]) {
      continue;
    }
    ++expanded;
    if (current.stable_index == *goal_node) {
      break;
    }
    for (const auto edge : graph_.nodes()[current.stable_index].edges) {
      const auto candidate = current.g + edge.weight_m;
      if (candidate < costs[edge.destination]) {
        if (!std::isfinite(costs[edge.destination])) {
          ++generated;
        }
        costs[edge.destination] = candidate;
        predecessor[edge.destination] = current.stable_index;
        const auto h = octile(graph_.nodes()[edge.destination].cell, goal,
                              grid_.resolution_m());
        open.push({candidate + h, h, candidate, edge.destination});
      }
    }
  }
  if (!std::isfinite(costs[*goal_node])) {
    auto result = reject(grid_, PlannerKind::graph, start, goal, RejectionReason::no_path);
    result.expanded_node_count = expanded;
    result.generated_node_count = generated;
    return result;
  }
  std::vector<GridIndex> reversed;
  auto cursor = *goal_node;
  for (std::size_t count = 0; count <= graph_.nodes().size(); ++count) {
    reversed.push_back(graph_.nodes()[cursor].cell);
    if (cursor == *start_node) {
      break;
    }
    if (!predecessor[cursor]) {
      return reject(grid_, PlannerKind::graph, start, goal,
                    RejectionReason::internal_invariant_failed);
    }
    cursor = *predecessor[cursor];
  }
  std::reverse(reversed.begin(), reversed.end());
  Plan2dResult result;
  result.status = PlanStatus::success;
  result.rejection_reason = RejectionReason::none;
  result.start = start;
  result.goal = goal;
  result.ordered_path = std::move(reversed);
  result.path_cost_m = costs[*goal_node];
  result.expanded_node_count = expanded;
  result.generated_node_count = generated;
  result.planner_kind = PlannerKind::graph;
  result.map_revision = grid_.revision();
  return result;
}

bool has_line_of_sight(const OccupancyGrid& grid, GridIndex from, GridIndex to,
                       UnknownPolicy policy) noexcept {
  if (!grid.in_bounds(from) || !grid.in_bounds(to) ||
      is_blocked(grid, from, policy) || is_blocked(grid, to, policy)) {
    return false;
  }
  auto x = from.column;
  auto y = from.row;
  const auto dx = to.column - from.column;
  const auto dy = to.row - from.row;
  const auto nx = std::abs(dx);
  const auto ny = std::abs(dy);
  const auto sign_x = dx > 0 ? 1 : (dx < 0 ? -1 : 0);
  const auto sign_y = dy > 0 ? 1 : (dy < 0 ? -1 : 0);
  std::int32_t ix = 0;
  std::int32_t iy = 0;
  while (ix < nx || iy < ny) {
    const auto left = (1 + 2 * ix) * ny;
    const auto right = (1 + 2 * iy) * nx;
    const auto previous_x = x;
    const auto previous_y = y;
    if (left == right) {
      x += sign_x;
      y += sign_y;
      ++ix;
      ++iy;
      if (is_blocked(grid, {previous_y, x}, policy) ||
          is_blocked(grid, {y, previous_x}, policy)) {
        return false;
      }
    } else if (left < right) {
      x += sign_x;
      ++ix;
    } else {
      y += sign_y;
      ++iy;
    }
    if (is_blocked(grid, {y, x}, policy)) {
      return false;
    }
  }
  return true;
}

PruneResult prune_path(const OccupancyGrid& grid, std::span<const GridIndex> path,
                       UnknownPolicy policy) {
  if (path.empty()) {
    return {RejectionReason::path_empty, {}};
  }
  for (const auto cell : path) {
    if (!grid.in_bounds(cell) || is_blocked(grid, cell, policy)) {
      return {RejectionReason::internal_invariant_failed, {}};
    }
  }
  std::vector<GridIndex> result{path.front()};
  std::size_t current = 0;
  while (current + 1 < path.size()) {
    auto furthest = path.size() - 1;
    while (furthest > current + 1 &&
           !has_line_of_sight(grid, path[current], path[furthest], policy)) {
      --furthest;
    }
    if (!has_line_of_sight(grid, path[current], path[furthest], policy)) {
      return {RejectionReason::internal_invariant_failed, {}};
    }
    result.push_back(path[furthest]);
    current = furthest;
  }
  return {RejectionReason::none, std::move(result)};
}

double path_length_m(const OccupancyGrid& grid,
                     std::span<const GridIndex> path) noexcept {
  double length = 0.0;
  for (std::size_t index = 1; index < path.size(); ++index) {
    const auto dr = static_cast<double>(path[index].row - path[index - 1].row);
    const auto dc = static_cast<double>(path[index].column - path[index - 1].column);
    length += std::hypot(dr, dc) * grid.resolution_m();
  }
  return length;
}

VoxelMap::VoxelMap(std::size_t width, std::size_t depth, std::size_t height,
                   double resolution_m, Position3d origin_m,
                   std::vector<Occupancy> voxels, std::uint64_t revision)
    : width_(width), depth_(depth), height_(height), resolution_m_(resolution_m),
      origin_m_(origin_m), voxels_(std::move(voxels)), revision_(revision) {
  const auto plane = checked_product(width_, depth_);
  const auto expected = checked_product(plane, height_);
  if (!(resolution_m_ > 0.0) || !std::isfinite(resolution_m_)) {
    throw std::invalid_argument("invalid resolution");
  }
  if (!finite(origin_m_) || voxels_.size() != expected ||
      !std::all_of(voxels_.begin(), voxels_.end(), valid_occupancy)) {
    throw std::invalid_argument("invalid voxel map");
  }
}

std::size_t VoxelMap::width() const noexcept { return width_; }
std::size_t VoxelMap::depth() const noexcept { return depth_; }
std::size_t VoxelMap::height() const noexcept { return height_; }
double VoxelMap::resolution_m() const noexcept { return resolution_m_; }
Position3d VoxelMap::origin_m() const noexcept { return origin_m_; }
std::uint64_t VoxelMap::revision() const noexcept { return revision_; }
const std::vector<Occupancy>& VoxelMap::voxels() const noexcept { return voxels_; }

bool VoxelMap::in_bounds(VoxelIndex index) const noexcept {
  return index.layer >= 0 && index.row >= 0 && index.column >= 0 &&
         static_cast<std::size_t>(index.layer) < height_ &&
         static_cast<std::size_t>(index.row) < depth_ &&
         static_cast<std::size_t>(index.column) < width_;
}

std::optional<Occupancy> VoxelMap::at(VoxelIndex index) const noexcept {
  if (!in_bounds(index)) {
    return std::nullopt;
  }
  return voxels_[linear(index, width_, depth_)];
}

std::optional<VoxelIndex> VoxelMap::world_to_voxel(Position3d point_m) const noexcept {
  if (!finite(point_m)) {
    return std::nullopt;
  }
  const auto column = std::floor((point_m.x_m - origin_m_.x_m) / resolution_m_);
  const auto row = std::floor((point_m.y_m - origin_m_.y_m) / resolution_m_);
  const auto layer = std::floor((point_m.z_m - origin_m_.z_m) / resolution_m_);
  if (column < 0.0 || row < 0.0 || layer < 0.0 ||
      column >= static_cast<double>(width_) || row >= static_cast<double>(depth_) ||
      layer >= static_cast<double>(height_)) {
    return std::nullopt;
  }
  return VoxelIndex{static_cast<std::int32_t>(layer), static_cast<std::int32_t>(row),
                    static_cast<std::int32_t>(column)};
}

std::optional<Position3d> VoxelMap::voxel_center(VoxelIndex index) const noexcept {
  if (!in_bounds(index)) {
    return std::nullopt;
  }
  return Position3d{
      origin_m_.x_m + (static_cast<double>(index.column) + 0.5) * resolution_m_,
      origin_m_.y_m + (static_cast<double>(index.row) + 0.5) * resolution_m_,
      origin_m_.z_m + (static_cast<double>(index.layer) + 0.5) * resolution_m_};
}

VoxelMap inflate_voxel_map(const VoxelMap& source, VehicleEnvelope3d envelope,
                           UnknownPolicy policy) {
  if (!std::isfinite(envelope.vehicle_radius_m) ||
      !std::isfinite(envelope.vehicle_half_height_m) ||
      !std::isfinite(envelope.horizontal_clearance_m) ||
      !std::isfinite(envelope.vertical_clearance_m) ||
      envelope.vehicle_radius_m < 0.0 || envelope.vehicle_half_height_m < 0.0 ||
      envelope.horizontal_clearance_m < 0.0 || envelope.vertical_clearance_m < 0.0) {
    throw std::invalid_argument("invalid 3D vehicle envelope");
  }
  const auto horizontal = envelope.vehicle_radius_m + envelope.horizontal_clearance_m;
  const auto vertical = envelope.vehicle_half_height_m + envelope.vertical_clearance_m;
  auto voxels = source.voxels();
  if (horizontal == 0.0 && vertical == 0.0) {
    if (policy == UnknownPolicy::blocked) {
      std::replace(voxels.begin(), voxels.end(), Occupancy::unknown, Occupancy::occupied);
    } else {
      std::replace(voxels.begin(), voxels.end(), Occupancy::unknown, Occupancy::free);
    }
    return VoxelMap(source.width(), source.depth(), source.height(), source.resolution_m(),
                    source.origin_m(), std::move(voxels), source.revision());
  }
  for (std::size_t layer = 0; layer < source.height(); ++layer) {
    for (std::size_t row = 0; row < source.depth(); ++row) {
      for (std::size_t column = 0; column < source.width(); ++column) {
        const VoxelIndex target{static_cast<std::int32_t>(layer),
                                static_cast<std::int32_t>(row),
                                static_cast<std::int32_t>(column)};
        bool blocked = false;
        for (std::size_t obstacle_layer = 0;
             obstacle_layer < source.height() && !blocked; ++obstacle_layer) {
          for (std::size_t obstacle_row = 0;
               obstacle_row < source.depth() && !blocked; ++obstacle_row) {
            for (std::size_t obstacle_column = 0; obstacle_column < source.width();
                 ++obstacle_column) {
              const VoxelIndex obstacle{static_cast<std::int32_t>(obstacle_layer),
                                        static_cast<std::int32_t>(obstacle_row),
                                        static_cast<std::int32_t>(obstacle_column)};
              const auto value = source.at(obstacle);
              if (!value || !policy_blocked(*value, policy)) {
                continue;
              }
              const auto dc = std::abs(target.column - obstacle.column);
              const auto dr = std::abs(target.row - obstacle.row);
              const auto dl = std::abs(target.layer - obstacle.layer);
              const auto dx = static_cast<double>(std::max(0, dc - 1)) * source.resolution_m();
              const auto dy = static_cast<double>(std::max(0, dr - 1)) * source.resolution_m();
              const auto dz = static_cast<double>(std::max(0, dl - 1)) * source.resolution_m();
              if (std::hypot(dx, dy) <= horizontal && dz <= vertical) {
                blocked = true;
                break;
              }
            }
          }
        }
        if (blocked) {
          voxels[linear(target, source.width(), source.depth())] = Occupancy::occupied;
        } else if (policy == UnknownPolicy::free &&
                   voxels[linear(target, source.width(), source.depth())] == Occupancy::unknown) {
          voxels[linear(target, source.width(), source.depth())] = Occupancy::free;
        }
      }
    }
  }
  return VoxelMap(source.width(), source.depth(), source.height(), source.resolution_m(),
                  source.origin_m(), std::move(voxels), source.revision());
}

PathValidationResult validate_path(const VoxelMap& source, const VoxelMap& collision_map,
                                   std::span<const TimedWaypoint> path,
                                   const PathValidationLimits& limits,
                                   UnknownPolicy policy) {
  PathValidationResult result;
  result.minimum_observed_clearance_m = std::numeric_limits<double>::infinity();
  const auto invalid_limit = [](const std::optional<double>& limit) {
    return limit && (!std::isfinite(*limit) || *limit < 0.0);
  };
  if (source.width() != collision_map.width() ||
      source.depth() != collision_map.depth() ||
      source.height() != collision_map.height() ||
      source.resolution_m() != collision_map.resolution_m() ||
      source.origin_m() != collision_map.origin_m()) {
    add_reason(result, RejectionReason::invalid_configuration);
    return result;
  }
  if (invalid_limit(limits.maximum_climb_rate_mps) ||
      invalid_limit(limits.maximum_waypoint_spacing_m) ||
      invalid_limit(limits.maximum_path_length_m) ||
      invalid_limit(limits.minimum_clearance_m)) {
    add_reason(result, RejectionReason::invalid_configuration);
    return result;
  }
  if (path.empty()) {
    add_reason(result, RejectionReason::path_empty);
    return result;
  }
  std::vector<std::optional<VoxelIndex>> indices(path.size());
  for (std::size_t index = 0; index < path.size(); ++index) {
    if (!finite(path[index].position_m) ||
        !std::isfinite(path[index].time_from_start.count())) {
      add_reason(result, RejectionReason::non_finite_value);
      if (!result.first_invalid_waypoint) {
        result.first_invalid_waypoint = index;
      }
      continue;
    }
    indices[index] = collision_map.world_to_voxel(path[index].position_m);
    if (!indices[index]) {
      add_reason(result, RejectionReason::waypoint_out_of_bounds);
      if (!result.first_invalid_waypoint) {
        result.first_invalid_waypoint = index;
      }
      continue;
    }
    const auto occupancy = collision_map.at(*indices[index]);
    if (!occupancy || policy_blocked(*occupancy, policy)) {
      add_reason(result, RejectionReason::waypoint_in_collision);
      if (!result.first_invalid_waypoint) {
        result.first_invalid_waypoint = index;
        result.first_colliding_voxel = indices[index];
      }
    }
    double clearance = std::numeric_limits<double>::infinity();
    const auto position = path[index].position_m;
    const auto origin = source.origin_m();
    const auto maximum = Position3d{
        origin.x_m + static_cast<double>(source.width()) * source.resolution_m(),
        origin.y_m + static_cast<double>(source.depth()) * source.resolution_m(),
        origin.z_m + static_cast<double>(source.height()) * source.resolution_m()};
    clearance = std::min({position.x_m - origin.x_m, maximum.x_m - position.x_m,
                          position.y_m - origin.y_m, maximum.y_m - position.y_m,
                          position.z_m - origin.z_m, maximum.z_m - position.z_m});
    for (std::size_t layer = 0; layer < source.height(); ++layer) {
      for (std::size_t row = 0; row < source.depth(); ++row) {
        for (std::size_t column = 0; column < source.width(); ++column) {
          const VoxelIndex obstacle{static_cast<std::int32_t>(layer),
                                    static_cast<std::int32_t>(row),
                                    static_cast<std::int32_t>(column)};
          const auto value = source.at(obstacle);
          if (value && policy_blocked(*value, policy)) {
            const auto centre = *source.voxel_center(obstacle);
            clearance = std::min(clearance, std::max(0.0, distance(position, centre) -
                std::sqrt(3.0) * source.resolution_m() * 0.5));
          }
        }
      }
    }
    result.minimum_observed_clearance_m =
        std::min(result.minimum_observed_clearance_m, clearance);
  }

  for (std::size_t index = 1; index < path.size(); ++index) {
    const auto segment_index = index - 1;
    if (finite(path[index - 1].position_m) && finite(path[index].position_m)) {
      const auto segment_length = distance(path[index - 1].position_m, path[index].position_m);
      result.maximum_observed_spacing_m =
          std::max(result.maximum_observed_spacing_m, segment_length);
      result.total_path_length_m += segment_length;
      if (segment_length == 0.0) {
        add_reason(result, RejectionReason::duplicate_waypoint);
        if (!result.first_invalid_segment) {
          result.first_invalid_segment = segment_index;
        }
      }
      if (limits.maximum_waypoint_spacing_m &&
          segment_length > *limits.maximum_waypoint_spacing_m) {
        add_reason(result, RejectionReason::waypoint_spacing_exceeded);
        if (!result.first_invalid_segment) {
          result.first_invalid_segment = segment_index;
        }
      }
      const auto samples = std::max<std::size_t>(
          1, static_cast<std::size_t>(std::ceil(segment_length /
              (collision_map.resolution_m() * 0.25))));
      for (std::size_t sample = 0; sample <= samples; ++sample) {
        const auto t = static_cast<double>(sample) / static_cast<double>(samples);
        const Position3d point{
            path[index - 1].position_m.x_m +
                t * (path[index].position_m.x_m - path[index - 1].position_m.x_m),
            path[index - 1].position_m.y_m +
                t * (path[index].position_m.y_m - path[index - 1].position_m.y_m),
            path[index - 1].position_m.z_m +
                t * (path[index].position_m.z_m - path[index - 1].position_m.z_m)};
        const auto voxel = collision_map.world_to_voxel(point);
        if (voxel) {
          const auto value = collision_map.at(*voxel);
          if (value && policy_blocked(*value, policy) &&
              (!indices[index - 1] || *voxel != *indices[index - 1]) &&
              (!indices[index] || *voxel != *indices[index])) {
            add_reason(result, RejectionReason::segment_in_collision);
            if (!result.first_invalid_segment) {
              result.first_invalid_segment = segment_index;
              result.first_colliding_voxel = voxel;
            }
            break;
          }
        }
      }
    }
    const auto delta_time =
        path[index].time_from_start.count() - path[index - 1].time_from_start.count();
    if (!(delta_time > 0.0) || !std::isfinite(delta_time)) {
      add_reason(result, RejectionReason::non_monotonic_time);
      if (!result.first_invalid_segment) {
        result.first_invalid_segment = segment_index;
      }
    } else if (limits.maximum_climb_rate_mps &&
               std::abs(path[index].position_m.z_m - path[index - 1].position_m.z_m) /
                       delta_time >
                   *limits.maximum_climb_rate_mps) {
      add_reason(result, RejectionReason::climb_rate_exceeded);
      if (!result.first_invalid_segment) {
        result.first_invalid_segment = segment_index;
      }
    }
  }
  if (limits.maximum_path_length_m &&
      result.total_path_length_m > *limits.maximum_path_length_m) {
    add_reason(result, RejectionReason::path_length_exceeded);
  }
  if (limits.minimum_clearance_m &&
      result.minimum_observed_clearance_m < *limits.minimum_clearance_m) {
    add_reason(result, RejectionReason::insufficient_clearance);
  }
  result.status = result.reasons.empty() ? ValidationStatus::valid : ValidationStatus::invalid;
  return result;
}

}  // namespace drone_lab::planning
