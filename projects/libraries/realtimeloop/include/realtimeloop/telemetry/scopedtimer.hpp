#pragma once

#include <realtime_loop/telemetry/loopinfo.hpp>
#include <cstdint>
#include <ctime>
#include <type_traits>

namespace Asclepius::Telemetry {

template <typename TelemetryBuffer, typename LoopTimer = void>
requires requires(TelemetryBuffer b, LoopTimer t) {
  { b.push };
  { t.get_schedule };
}
class ScopedTimer {
public:
  ScopedTimer(TelemetryBuffer &buf, LoopTimer &timer) : m_buf{buf}, m_timer{timer} {
    clock_gettime(CLOCK_MONOTONIC, &m_t_creation);
  }

  ~ScopedTimer() {
    struct timespec t_destruction;
    clock_gettime(CLOCK_MONOTONIC, &t_destruction);
    LoopInfo loop_info{timespec_to_us(m_timer.get_schedule()), timespec_to_us(m_t_creation),
                       timespec_to_us(t_destruction)};
    m_buf.push(loop_info);
  }

private:
  constexpr static int64_t ONE_SEC_IN_NS = 1'000'000'000;
  constexpr static int64_t ONE_SEC_IN_US = 1'000'000;
  constexpr static int64_t ONE_US_IN_NS = 1'000;

  static int64_t timespec_to_us(struct timespec ts) {
    return ts.tv_nsec / ONE_US_IN_NS + ts.tv_sec * ONE_SEC_IN_US;
  }

  struct timespec m_t_creation;
  TelemetryBuffer &m_buf;
  LoopTimer &m_timer;
}; // class ScopedTimer

template <typename TelemetryBuffer> class ScopedTimer<TelemetryBuffer, void> {
public:
  ScopedTimer(TelemetryBuffer &buf) : m_buf{buf} { clock_gettime(CLOCK_MONOTONIC, &m_t_creation); }

  ~ScopedTimer() {
    struct timespec t_destruction;
    clock_gettime(CLOCK_MONOTONIC, &t_destruction);
    LoopInfo info{
        .expected_wake_us = -1,
        .actual_wake_us = timespec_to_us(m_t_creation),
        .loop_end_us = timespec_to_us(t_destruction),
    };
    m_buf.push(info);
  }

private:
  constexpr static int64_t ONE_SEC_IN_NS = 1'000'000'000;
  constexpr static int64_t ONE_SEC_IN_US = 1'000'000;
  constexpr static int64_t ONE_US_IN_NS = 1'000;

  static int64_t timespec_to_us(struct timespec ts) {
    return ts.tv_nsec / ONE_US_IN_NS + ts.tv_sec * ONE_SEC_IN_US;
  }
  struct timespec m_t_creation;
  TelemetryBuffer &m_buf;
}; // class ScopedTimer

struct NullTimer {
  template <typename... Args> NullTimer(Args &&...) {}
};

template <bool Enabled, typename TelemetryBuffer, typename LoopTimer>
using ConditionalTimer =
    std::conditional_t<Enabled, ScopedTimer<TelemetryBuffer, LoopTimer>, NullTimer>;

}; // namespace Asclepius::Telemetry
