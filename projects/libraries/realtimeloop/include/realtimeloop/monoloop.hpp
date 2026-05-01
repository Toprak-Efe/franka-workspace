#pragma once

#include <cstring>
#include <ctime>
#include <functional>
#include <realtimeloop/asynclogger.hpp>
#include <realtimeloop/schedtimer.hpp>
#include <stdexcept>
#include <stop_token>
#include <sys/mman.h>

namespace Asclepius {

template <int64_t NsPeriod>
requires(NsPeriod > 0)
class Monoloop {
public:
  Monoloop(std::stop_token stoken, std::function<void()> &&func) {
    Schedtimer<NsPeriod> loop_timer;
    get_realtime_priority();

    for (;;) {
      {
        if (stoken.stop_requested()) {
          return;
        }
        func();
      }
      loop_timer.await();
    }
  }

private:
  void get_realtime_priority() {
    const int thread_priority = sched_get_priority_max(SCHED_FIFO);
    if (thread_priority == -1) {
      throw std::runtime_error("Asclepius: unable to get maximum priority for realtime loop.");
    }

    sched_param thread_param{};
    thread_param.sched_priority = thread_priority;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param) != 0) {
      throw std::runtime_error("Asclepius: unable to set realtime scheduling for realtime loop.");
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0) {
      throw std::runtime_error("Asclepius: unable to lock memory pages for realtime loop: " +
                               std::to_string(errno));
    }
  }
}; // class Monoloop

}; // namespace Asclepius
