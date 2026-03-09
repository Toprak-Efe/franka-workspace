#pragma once

#include <realtime_loop/schedtimer.hpp>
#include <realtime_loop/telemetry/asynclogger.hpp>
#include <realtime_loop/telemetry/scopedtimer.hpp>
#include <atomic>
#include <chrono>
#include <concepts>
#include <cstring>
#include <ctime>
#include <stdexcept>
#include <type_traits>

namespace Asclepius {

template <auto LoopPeriod, bool TelemetryEnabled = false>
requires requires { std::chrono::duration_cast<std::chrono::nanoseconds>(LoopPeriod); }
class Monoloop {
public:
  Monoloop() : m_running{false} {}
  ~Monoloop() { stop(); }

  template <typename F>
  requires std::invocable<F>
  void start(F &&func) {
    if (m_running.exchange(true, std::memory_order_relaxed))
      return;
    constexpr auto ns_dur = std::chrono::duration_cast<std::chrono::nanoseconds>(LoopPeriod);
    constexpr int64_t period = ns_dur.count();
    Schedtimer<period> loop_timer;

    const int thread_priority = sched_get_priority_max(SCHED_FIFO);
    if (thread_priority == -1) {
      throw std::runtime_error("Asclepius: unable to get maximum priority for realtime loop.");
    }

    sched_param thread_param{};
    thread_param.sched_priority = thread_priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0) {
      throw std::runtime_error("Asclepius: unable to set realtime scheduling for realtime loop.");
    }

    for (;;) {
      {
        Telemetry::ConditionalTimer<TelemetryEnabled, decltype(m_loop_log), decltype(loop_timer)>
            guard{m_loop_log};
        if (!m_running.load(std::memory_order_relaxed))
          break;
        func();
      }
      loop_timer.await();
    }
  }

  void stop() {
    m_running.store(false, std::memory_order_relaxed);
    if constexpr (TelemetryEnabled) {
      m_loop_log.stop();
    }
  }

private:
  struct _Empty {};
  [[no_unique_address]] std::conditional_t<TelemetryEnabled, Telemetry::AsyncLogger<double, 512>,
                                           _Empty> m_loop_log;
  std::atomic_bool m_running;

}; // class Monoloop

}; // namespace Asclepius
