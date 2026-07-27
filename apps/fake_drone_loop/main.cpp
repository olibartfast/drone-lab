#include <fstream>
#include <iostream>
#include <string>

#include "drone_lab/simulation/fake_drone_loop.hpp"

int main(int argc, char** argv) {
  const std::string output_path = argc > 1 ? argv[1] : "fake-drone-loop.jsonl";
  std::ofstream output(output_path);
  if (!output) {
    std::cerr << "cannot open output log: " << output_path << '\n';
    return 2;
  }

  const drone_lab::FakeDroneLoop loop;
  const auto metrics = loop.run(output);
  std::cout << "ticks=" << metrics.ticks
            << " mean_error_px=" << metrics.mean_error_px
            << " final_error_px=" << metrics.final_error_px
            << " accepted=" << metrics.accepted_commands
            << " rejected=" << metrics.rejected_commands
            << " unsafe=" << (metrics.unsafe_command_accepted ? 1 : 0)
            << '\n';

  return metrics.unsafe_command_accepted ? 1 : 0;
}
