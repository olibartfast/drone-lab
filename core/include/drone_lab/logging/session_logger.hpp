#pragma once

#include <ostream>
#include <string>
#include <string_view>
#include "drone_lab/platform/types.hpp"

namespace drone_lab {
class SessionLogger {
 public:
  SessionLogger(std::ostream& output, std::string_view session_id);
  void log(TimestampNs timestamp_ns, std::string_view component, std::string_view severity,
           std::string_view event, std::string_view message = {});
 private:
  std::ostream& output_;
  std::string session_id_;
};
}  // namespace drone_lab
