#pragma once

#include <cstdint>

namespace Asclepius::Telemetry {

struct LoopInfo {
  int64_t expected_wake_us;
  int64_t actual_wake_us;
  int64_t loop_end_us;
}; // struct LoopInfo

}; // namespace Asclepius::Telemetry

