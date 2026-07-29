#include "drone_lab/planning/planning.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace {
using drone_lab::planning::GraphAStarPlanner;
using drone_lab::planning::GridAStarPlanner;
using drone_lab::planning::GridIndex;
using drone_lab::planning::Occupancy;
using drone_lab::planning::OccupancyGrid;
using drone_lab::planning::PlanStatus;
using drone_lab::planning::Planner2d;
using drone_lab::planning::PlannerKind;
using drone_lab::planning::Position2d;
using drone_lab::planning::RejectionReason;
using drone_lab::planning::UnknownPolicy;

struct GridFixture {
  std::string scenario;
  OccupancyGrid grid;
  UnknownPolicy unknown_policy;
  double vehicle_radius_m;
  double clearance_m;
  GridIndex start;
  GridIndex goal;
  PlanStatus expected_status;
  RejectionReason expected_reason;
};

struct VoxelFixture {
  std::string scenario;
  drone_lab::planning::VoxelMap map;
  UnknownPolicy unknown_policy;
  drone_lab::planning::VehicleEnvelope3d envelope;
  std::vector<drone_lab::planning::TimedWaypoint> waypoints;
  drone_lab::planning::PathValidationLimits limits;
  drone_lab::planning::ValidationStatus expected_status;
  RejectionReason expected_reason;
};

using Fixture = std::variant<GridFixture, VoxelFixture>;

struct ParseFailure : std::runtime_error {
  ParseFailure(RejectionReason reason_value, const std::string& message)
      : std::runtime_error(message), reason(reason_value) {}
  RejectionReason reason;
};

[[nodiscard]] std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return {};
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

template <typename Integer>
[[nodiscard]] Integer parse_integer(const std::string& text) {
  Integer value{};
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
    throw ParseFailure(RejectionReason::malformed_fixture, "invalid integer: " + text);
  }
  return value;
}

[[nodiscard]] double parse_double(const std::string& text) {
  std::istringstream input(text);
  input.imbue(std::locale::classic());
  double value{};
  input >> value;
  if (!input || input.peek() != std::char_traits<char>::eof() || !std::isfinite(value)) {
    throw ParseFailure(RejectionReason::non_finite_value, "invalid finite number: " + text);
  }
  return value;
}

[[nodiscard]] std::vector<std::string> split(const std::string& text, char separator) {
  std::vector<std::string> parts;
  std::istringstream input(text);
  std::string part;
  while (std::getline(input, part, separator)) {
    parts.push_back(trim(part));
  }
  return parts;
}

[[nodiscard]] const std::string& required(const std::map<std::string, std::string>& values,
                                          const std::string& key) {
  const auto found = values.find(key);
  if (found == values.end()) {
    throw ParseFailure(RejectionReason::malformed_fixture, "missing key: " + key);
  }
  return found->second;
}

[[nodiscard]] RejectionReason parse_expected_reason(const std::string& text) {
  constexpr std::array<std::pair<std::string_view, RejectionReason>, 13> reasons{{
      {"none", RejectionReason::none},
      {"no_path", RejectionReason::no_path},
      {"waypoint_out_of_bounds", RejectionReason::waypoint_out_of_bounds},
      {"waypoint_in_collision", RejectionReason::waypoint_in_collision},
      {"segment_in_collision", RejectionReason::segment_in_collision},
      {"insufficient_clearance", RejectionReason::insufficient_clearance},
      {"duplicate_waypoint", RejectionReason::duplicate_waypoint},
      {"non_monotonic_time", RejectionReason::non_monotonic_time},
      {"climb_rate_exceeded", RejectionReason::climb_rate_exceeded},
      {"waypoint_spacing_exceeded", RejectionReason::waypoint_spacing_exceeded},
      {"path_length_exceeded", RejectionReason::path_length_exceeded},
      {"non_finite_value", RejectionReason::non_finite_value},
      {"invalid_configuration", RejectionReason::invalid_configuration},
  }};
  const auto found = std::find_if(reasons.begin(), reasons.end(), [&](const auto& entry) {
    return entry.first == text;
  });
  if (found == reasons.end()) {
    throw ParseFailure(RejectionReason::malformed_fixture, "unsupported expected reason");
  }
  return found->second;
}

[[nodiscard]] std::optional<double> parse_optional_limit(const std::string& text) {
  return text == "none" ? std::nullopt : std::optional<double>(parse_double(text));
}

[[nodiscard]] Fixture parse_fixture(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    throw ParseFailure(RejectionReason::malformed_fixture, "fixture is unreadable");
  }
  std::map<std::string, std::string> values;
  std::vector<std::string> rows;
  std::string line;
  bool reading_rows = false;
  while (std::getline(input, line)) {
    line = trim(line);
    if (line.empty() || line.starts_with("//")) {
      continue;
    }
    if (reading_rows) {
      rows.push_back(line);
      continue;
    }
    if (line == "data:") {
      reading_rows = true;
      continue;
    }
    const auto separator = line.find('=');
    if (separator == std::string::npos) {
      throw ParseFailure(RejectionReason::malformed_fixture, "expected key=value");
    }
    const auto key = trim(line.substr(0, separator));
    if (key.empty() || values.contains(key)) {
      throw ParseFailure(RejectionReason::malformed_fixture, "empty or duplicate key");
    }
    values.emplace(key, trim(line.substr(separator + 1)));
  }
  if (required(values, "version") != "1") {
    throw ParseFailure(RejectionReason::unsupported_fixture_version,
                       "unsupported fixture version");
  }
  const auto map_kind = required(values, "map_kind");
  const auto width = parse_integer<std::size_t>(required(values, "width"));
  auto parse_occupancy = [&](std::size_t expected_rows) {
    std::vector<Occupancy> cells;
    cells.reserve(width * expected_rows);
    std::size_t actual_rows = 0;
    for (const auto& row : rows) {
      if (row == "---") {
        continue;
      }
      ++actual_rows;
      if (row.size() != width) {
        throw ParseFailure(RejectionReason::invalid_map_dimensions, "row width mismatch");
      }
      for (const char symbol : row) {
        switch (symbol) {
          case '.': cells.push_back(Occupancy::free); break;
          case '#': cells.push_back(Occupancy::occupied); break;
          case '?': cells.push_back(Occupancy::unknown); break;
          default:
            throw ParseFailure(RejectionReason::malformed_fixture, "unknown occupancy symbol");
        }
      }
    }
    if (actual_rows != expected_rows) {
      throw ParseFailure(RejectionReason::invalid_map_dimensions, "row count mismatch");
    }
    return cells;
  };
  const auto policy_text = required(values, "unknown_policy");
  const auto policy = policy_text == "blocked" ? UnknownPolicy::blocked :
                      policy_text == "free" ? UnknownPolicy::free :
                      throw ParseFailure(RejectionReason::malformed_fixture,
                                         "unknown policy must be blocked or free");
  const auto expected_reason = parse_expected_reason(required(values, "expected_reason"));
  if (map_kind == "grid2d") {
    const auto height = parse_integer<std::size_t>(required(values, "height"));
    auto cells = parse_occupancy(height);
    const auto origin_parts = split(required(values, "origin_m"), ',');
    const auto start_parts = split(required(values, "start"), ',');
    const auto goal_parts = split(required(values, "goal"), ',');
    if (origin_parts.size() != 2 || start_parts.size() != 2 || goal_parts.size() != 2) {
      throw ParseFailure(RejectionReason::malformed_fixture, "invalid coordinate tuple");
    }
    const auto expected_text = required(values, "expected_status");
    const auto expected_status = expected_text == "success" ? PlanStatus::success :
                                 expected_text == "rejected" ? PlanStatus::rejected :
                                 throw ParseFailure(RejectionReason::malformed_fixture,
                                                    "invalid expected status");
    try {
      return GridFixture{
          required(values, "scenario"),
          OccupancyGrid(width, height, parse_double(required(values, "resolution_m")),
                        {parse_double(origin_parts[0]), parse_double(origin_parts[1])},
                        std::move(cells),
                        parse_integer<std::uint64_t>(required(values, "revision"))),
          policy,
          parse_double(required(values, "vehicle_radius_m")),
          parse_double(required(values, "clearance_m")),
          {parse_integer<std::int32_t>(start_parts[0]),
           parse_integer<std::int32_t>(start_parts[1])},
          {parse_integer<std::int32_t>(goal_parts[0]),
           parse_integer<std::int32_t>(goal_parts[1])},
          expected_status,
          expected_reason};
    } catch (const std::invalid_argument& error) {
      throw ParseFailure(RejectionReason::invalid_configuration, error.what());
    }
  }
  if (map_kind != "voxel3d") {
    throw ParseFailure(RejectionReason::malformed_fixture,
                       "map_kind must be grid2d or voxel3d");
  }
  const auto depth = parse_integer<std::size_t>(required(values, "depth"));
  const auto height = parse_integer<std::size_t>(required(values, "height"));
  auto voxels = parse_occupancy(depth * height);
  const auto origin_parts = split(required(values, "origin_m"), ',');
  if (origin_parts.size() != 3) {
    throw ParseFailure(RejectionReason::malformed_fixture, "invalid voxel origin");
  }
  std::vector<drone_lab::planning::TimedWaypoint> waypoints;
  for (const auto& waypoint_text : split(required(values, "waypoints"), ';')) {
    const auto parts = split(waypoint_text, ',');
    if (parts.size() != 4) {
      throw ParseFailure(RejectionReason::malformed_fixture, "invalid timed waypoint");
    }
    waypoints.push_back({
        {parse_double(parts[0]), parse_double(parts[1]), parse_double(parts[2])},
        std::chrono::duration<double>(parse_double(parts[3]))});
  }
  const auto expected_text = required(values, "expected_status");
  const auto expected_status =
      expected_text == "valid" ? drone_lab::planning::ValidationStatus::valid :
      expected_text == "invalid" ? drone_lab::planning::ValidationStatus::invalid :
      throw ParseFailure(RejectionReason::malformed_fixture, "invalid validation status");
  try {
    return VoxelFixture{
        required(values, "scenario"),
        drone_lab::planning::VoxelMap(
            width, depth, height, parse_double(required(values, "resolution_m")),
            {parse_double(origin_parts[0]), parse_double(origin_parts[1]),
             parse_double(origin_parts[2])},
            std::move(voxels),
            parse_integer<std::uint64_t>(required(values, "revision"))),
        policy,
        {parse_double(required(values, "vehicle_radius_m")),
         parse_double(required(values, "vehicle_half_height_m")),
         parse_double(required(values, "horizontal_clearance_m")),
         parse_double(required(values, "vertical_clearance_m"))},
        std::move(waypoints),
        {parse_optional_limit(required(values, "maximum_climb_rate_mps")),
         parse_optional_limit(required(values, "maximum_waypoint_spacing_m")),
         parse_optional_limit(required(values, "maximum_path_length_m")),
         parse_optional_limit(required(values, "minimum_clearance_m"))},
        expected_status,
        expected_reason};
  } catch (const std::invalid_argument& error) {
    throw ParseFailure(RejectionReason::invalid_configuration, error.what());
  }
}

void write_index(std::ostream& output, GridIndex index) {
  output << "{\"row\":" << index.row << ",\"column\":" << index.column << '}';
}

void write_path(std::ostream& output, const std::vector<GridIndex>& path) {
  output << '[';
  for (std::size_t index = 0; index < path.size(); ++index) {
    if (index != 0) {
      output << ',';
    }
    write_index(output, path[index]);
  }
  output << ']';
}

void write_blocked_cells(std::ostream& output, const OccupancyGrid& grid,
                         UnknownPolicy policy) {
  output << '[';
  bool first = true;
  for (std::size_t row = 0; row < grid.height(); ++row) {
    for (std::size_t column = 0; column < grid.width(); ++column) {
      const GridIndex cell{static_cast<std::int32_t>(row),
                           static_cast<std::int32_t>(column)};
      if (!drone_lab::planning::is_blocked(grid, cell, policy)) {
        continue;
      }
      if (!first) {
        output << ',';
      }
      first = false;
      write_index(output, cell);
    }
  }
  output << ']';
}

[[nodiscard]] std::string failure_json(RejectionReason reason, std::string_view message) {
  std::ostringstream output;
  output << "{\"schema_version\":1,\"status\":\"error\",\"rejection_reason\":\""
         << drone_lab::planning::to_string(reason) << "\",\"message\":\"";
  for (const char character : message) {
    if (character == '"' || character == '\\') {
      output << '\\';
    }
    output << character;
  }
  output << "\"}\n";
  return output.str();
}

void usage(std::ostream& output) {
  output << "usage: planner_lab --fixture PATH --planner grid|graph [--output PATH]\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::optional<std::filesystem::path> fixture_path;
  std::optional<std::filesystem::path> output_path;
  std::optional<PlannerKind> planner_kind;
  for (int index = 1; index < argc; ++index) {
    const std::string_view argument(argv[index]);
    if (argument == "--help") {
      usage(std::cout);
      return 0;
    }
    if (argument == "--fixture" || argument == "--planner" || argument == "--output") {
      if (index + 1 >= argc) {
        usage(std::cerr);
        return 2;
      }
      const std::string value(argv[++index]);
      if (argument == "--fixture" && !fixture_path) {
        fixture_path = value;
      } else if (argument == "--output" && !output_path) {
        output_path = value;
      } else if (argument == "--planner" && !planner_kind) {
        if (value == "grid") {
          planner_kind = PlannerKind::grid;
        } else if (value == "graph") {
          planner_kind = PlannerKind::graph;
        } else {
          usage(std::cerr);
          return 2;
        }
      } else {
        usage(std::cerr);
        return 2;
      }
    } else {
      usage(std::cerr);
      return 2;
    }
  }
  if (!fixture_path || !planner_kind) {
    usage(std::cerr);
    return 2;
  }

  try {
    const auto parsed_fixture = parse_fixture(*fixture_path);
    if (std::holds_alternative<VoxelFixture>(parsed_fixture)) {
      const auto& fixture = std::get<VoxelFixture>(parsed_fixture);
      const auto collision_map = drone_lab::planning::inflate_voxel_map(
          fixture.map, fixture.envelope, fixture.unknown_policy);
      const auto validation = drone_lab::planning::validate_path(
          fixture.map, collision_map, fixture.waypoints, fixture.limits,
          fixture.unknown_policy);
      const auto expected_reason_present =
          fixture.expected_reason == RejectionReason::none
              ? validation.reasons.empty()
              : std::find(validation.reasons.begin(), validation.reasons.end(),
                          fixture.expected_reason) != validation.reasons.end();
      const auto expectation_matches =
          validation.status == fixture.expected_status && expected_reason_present;
      const auto blocked_count = [&](const drone_lab::planning::VoxelMap& map) {
        return static_cast<std::size_t>(std::count_if(
            map.voxels().begin(), map.voxels().end(), [&](Occupancy value) {
              return value == Occupancy::occupied ||
                     (value == Occupancy::unknown &&
                      fixture.unknown_policy == UnknownPolicy::blocked);
            }));
      };
      std::ostringstream report;
      report.imbue(std::locale::classic());
      report << std::fixed << std::setprecision(6);
      report << "{\"schema_version\":1,\"scenario\":\"" << fixture.scenario
             << "\",\"map_kind\":\"voxel3d\",\"map_revision\":"
             << fixture.map.revision() << ",\"map\":{\"width\":" << fixture.map.width()
             << ",\"depth\":" << fixture.map.depth()
             << ",\"height\":" << fixture.map.height()
             << ",\"resolution_m\":" << fixture.map.resolution_m()
             << ",\"origin_m\":{\"x\":" << fixture.map.origin_m().x_m
             << ",\"y\":" << fixture.map.origin_m().y_m
             << ",\"z\":" << fixture.map.origin_m().z_m << "}}"
             << ",\"planner_kind\":\"" << drone_lab::planning::to_string(*planner_kind)
             << "\",\"unknown_policy\":\""
             << drone_lab::planning::to_string(fixture.unknown_policy)
             << "\",\"start\":null,\"goal\":null,\"vehicle_envelope\":{"
             << "\"vehicle_radius_m\":" << fixture.envelope.vehicle_radius_m
             << ",\"vehicle_half_height_m\":" << fixture.envelope.vehicle_half_height_m
             << ",\"horizontal_clearance_m\":"
             << fixture.envelope.horizontal_clearance_m
             << ",\"vertical_clearance_m\":" << fixture.envelope.vertical_clearance_m
             << "},\"status\":\""
             << (validation.status == drone_lab::planning::ValidationStatus::valid
                     ? "success"
                     : "rejected")
             << "\",\"rejection_reason\":\""
             << (validation.reasons.empty()
                     ? drone_lab::planning::to_string(RejectionReason::none)
                     : drone_lab::planning::to_string(validation.reasons.front()))
             << "\",\"source_occupied_count\":" << blocked_count(fixture.map)
             << ",\"inflated_occupied_count\":" << blocked_count(collision_map)
             << ",\"expanded_node_count\":0,\"generated_node_count\":0"
             << ",\"raw_waypoint_count\":" << fixture.waypoints.size()
             << ",\"pruned_waypoint_count\":null,\"raw_path_cost_m\":null"
             << ",\"pruned_path_length_m\":null,\"minimum_clearance_m\":";
      if (std::isfinite(validation.minimum_observed_clearance_m)) {
        report << validation.minimum_observed_clearance_m;
      } else {
        report << "null";
      }
      report << ",\"validation_status\":\""
             << drone_lab::planning::to_string(validation.status)
             << "\",\"validation_reasons\":[";
      for (std::size_t index = 0; index < validation.reasons.size(); ++index) {
        if (index != 0) {
          report << ',';
        }
        report << '"' << drone_lab::planning::to_string(validation.reasons[index]) << '"';
      }
      report << "],\"first_invalid_waypoint\":";
      if (validation.first_invalid_waypoint) {
        report << *validation.first_invalid_waypoint;
      } else {
        report << "null";
      }
      report << ",\"first_invalid_segment\":";
      if (validation.first_invalid_segment) {
        report << *validation.first_invalid_segment;
      } else {
        report << "null";
      }
      report << ",\"maximum_observed_spacing_m\":"
             << validation.maximum_observed_spacing_m
             << ",\"total_path_length_m\":" << validation.total_path_length_m
             << ",\"raw_path\":[],\"pruned_path\":[]"
             << ",\"source_blocked_cells\":[],\"inflated_blocked_cells\":[]"
             << ",\"expectation_matches\":"
             << (expectation_matches ? "true" : "false") << "}\n";
      if (output_path) {
        std::ofstream output(*output_path, std::ios::binary | std::ios::trunc);
        if (!output || !(output << report.str())) {
          std::cerr << failure_json(RejectionReason::output_write_failed,
                                    "could not write output");
          return 1;
        }
      } else {
        std::cout << report.str();
      }
      return expectation_matches ? 0 : 1;
    }
    const auto& fixture = std::get<GridFixture>(parsed_fixture);
    const auto inflated = drone_lab::planning::inflate_grid(
        fixture.grid, fixture.vehicle_radius_m, fixture.clearance_m,
        fixture.unknown_policy);
    std::unique_ptr<Planner2d> planner;
    if (*planner_kind == PlannerKind::grid) {
      planner = std::make_unique<GridAStarPlanner>(inflated.grid, fixture.unknown_policy);
    } else {
      planner = std::make_unique<GraphAStarPlanner>(inflated.grid, fixture.unknown_policy);
    }
    const auto result = planner->plan(fixture.start, fixture.goal);
    const auto pruned = result.status == PlanStatus::success
                            ? drone_lab::planning::prune_path(
                                  inflated.grid, result.ordered_path, fixture.unknown_policy)
                            : drone_lab::planning::PruneResult{result.rejection_reason, {}};
    const auto expectation_matches =
        result.status == fixture.expected_status &&
        result.rejection_reason == fixture.expected_reason &&
        (result.status != PlanStatus::success ||
         pruned.rejection_reason == RejectionReason::none);

    std::ostringstream report;
    report.imbue(std::locale::classic());
    report << std::fixed << std::setprecision(6);
    report << "{\"schema_version\":1,\"scenario\":\"" << fixture.scenario
           << "\",\"map_kind\":\"grid2d\",\"map_revision\":" << fixture.grid.revision()
           << ",\"map\":{\"width\":" << fixture.grid.width()
           << ",\"height\":" << fixture.grid.height()
           << ",\"resolution_m\":" << fixture.grid.resolution_m()
           << ",\"origin_m\":{\"x\":" << fixture.grid.origin_m().x_m
           << ",\"y\":" << fixture.grid.origin_m().y_m << "}}"
           << ",\"planner_kind\":\"" << drone_lab::planning::to_string(*planner_kind)
           << "\",\"unknown_policy\":\""
           << drone_lab::planning::to_string(fixture.unknown_policy) << "\",\"start\":";
    write_index(report, fixture.start);
    report << ",\"goal\":";
    write_index(report, fixture.goal);
    report << ",\"vehicle_envelope\":{\"vehicle_radius_m\":" << fixture.vehicle_radius_m
           << ",\"clearance_m\":" << fixture.clearance_m << "},\"status\":\""
           << drone_lab::planning::to_string(result.status)
           << "\",\"rejection_reason\":\""
           << drone_lab::planning::to_string(result.rejection_reason)
           << "\",\"source_occupied_count\":"
           << drone_lab::planning::blocked_cell_count(
                  fixture.grid, fixture.unknown_policy)
           << ",\"inflated_occupied_count\":"
           << drone_lab::planning::blocked_cell_count(
                  inflated.grid, fixture.unknown_policy)
           << ",\"expanded_node_count\":" << result.expanded_node_count
           << ",\"generated_node_count\":" << result.generated_node_count
           << ",\"raw_waypoint_count\":" << result.ordered_path.size()
           << ",\"pruned_waypoint_count\":" << pruned.path.size()
           << ",\"raw_path_cost_m\":" << result.path_cost_m
           << ",\"pruned_path_length_m\":"
           << drone_lab::planning::path_length_m(inflated.grid, pruned.path)
           << ",\"minimum_clearance_m\":null,\"validation_status\":\""
           << (pruned.rejection_reason == RejectionReason::none ? "valid" : "invalid")
           << "\",\"validation_reasons\":[";
    if (pruned.rejection_reason != RejectionReason::none) {
      report << '"' << drone_lab::planning::to_string(pruned.rejection_reason) << '"';
    }
    report << "],\"raw_path\":";
    write_path(report, result.ordered_path);
    report << ",\"pruned_path\":";
    write_path(report, pruned.path);
    report << ",\"source_blocked_cells\":";
    write_blocked_cells(report, fixture.grid, fixture.unknown_policy);
    report << ",\"inflated_blocked_cells\":";
    write_blocked_cells(report, inflated.grid, fixture.unknown_policy);
    report << ",\"expectation_matches\":" << (expectation_matches ? "true" : "false")
           << "}\n";

    if (output_path) {
      std::ofstream output(*output_path, std::ios::binary | std::ios::trunc);
      if (!output || !(output << report.str())) {
        std::cerr << failure_json(RejectionReason::output_write_failed,
                                  "could not write output") ;
        return 1;
      }
    } else {
      std::cout << report.str();
    }
    return expectation_matches ? 0 : 1;
  } catch (const ParseFailure& error) {
    std::cerr << failure_json(error.reason, error.what());
    return 1;
  } catch (const std::exception& error) {
    std::cerr << failure_json(RejectionReason::internal_invariant_failed, error.what());
    return 1;
  }
}
