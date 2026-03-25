#pragma once

#include <concepts>
#include <cstring>
#include <ctime>
#include <realtimeloop/asynclogger.hpp>
#include <realtimeloop/schedtimer.hpp>
#include <realtimeloop/scopedtimer.hpp>
#include <stdexcept>
#include <stop_token>
#include <type_traits>

namespace Asclepius {

template <typename F, int64_t NsPeriod, bool TelemetryEnabled = false>
requires requires {
  { NsPeriod > 0 };
  { std::invocable<F> };
}
class Monoloop {
public:
  Monoloop(std::stop_token &stoken, F &&func) {
    Schedtimer<NsPeriod> loop_timer;

    const int thread_priority = sched_get_priority_max(SCHED_FIFO);
    if (thread_priority == -1) {
      throw std::runtime_error("Asclepius: unable to get maximum priority for realtime loop.");
    }

    sched_param thread_param{};
    thread_param.sched_priority = thread_priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0) {
      throw std::runtime_error("Asclepius: unable to set realtime scheduling for realtime loop.");
    }

    if constexpr (TelemetryEnabled) {
      m_loop_log.start();
    }

    for (;;) {
      {
        Telemetry::ConditionalTimer<TelemetryEnabled, decltype(m_loop_log), decltype(loop_timer)>
            guard{m_loop_log};
        if (stoken.stop_requested())
          return;
        func();
      }
      loop_timer.await();
    }

    if constexpr (TelemetryEnabled) {
      m_loop_log.stop();
    }
  }

private:
  struct _Empty {};
  [[no_unique_address]] std::conditional_t<TelemetryEnabled, Telemetry::AsyncLogger<double, 512>,
                                           _Empty> m_loop_log;
}; // class Monoloop

}; // namespace Asclepius
