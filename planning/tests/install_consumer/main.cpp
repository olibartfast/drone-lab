#include "drone_lab/planning/planning.hpp"

int main() {
  const drone_lab::planning::OccupancyGrid grid(
      1, 1, 1.0, {}, {drone_lab::planning::Occupancy::free});
  const drone_lab::planning::GridAStarPlanner planner(grid);
  return planner.plan({0, 0}, {0, 0}).status ==
                 drone_lab::planning::PlanStatus::success
             ? 0
             : 1;
}
