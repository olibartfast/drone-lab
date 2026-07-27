#include "drone_lab/logging/session_logger.hpp"

namespace drone_lab {
namespace {
void write_json_string(std::ostream& out, std::string_view value) {
  out << '"';
  for (const char c : value) {
    if (c == '"' || c == '\\') out << '\\';
    if (c == '\n') out << "\\n"; else out << c;
  }
  out << '"';
}
}
SessionLogger::SessionLogger(std::ostream& output, std::string_view session_id)
    : output_(output), session_id_(session_id) {}
void SessionLogger::log(TimestampNs timestamp_ns, std::string_view component,
                        std::string_view severity, std::string_view event,
                        std::string_view message) {
  output_ << "{\"timestamp_ns\":" << timestamp_ns << ",\"session_id\":";
  write_json_string(output_, session_id_);
  output_ << ",\"component\":"; write_json_string(output_, component);
  output_ << ",\"severity\":"; write_json_string(output_, severity);
  output_ << ",\"event\":"; write_json_string(output_, event);
  output_ << ",\"message\":"; write_json_string(output_, message);
  output_ << "}\n";
}
}  // namespace drone_lab
