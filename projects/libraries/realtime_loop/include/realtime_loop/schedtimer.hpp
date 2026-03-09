#pragma once

#include <cstdio>
#include <ctime>
#include <stdint.h>

namespace Asclepius {

template <int64_t T>
requires(T > 0)
class Schedtimer {
public:
  Schedtimer() { clock_gettime(CLOCK_MONOTONIC, &m_schedule); }

  void await() {
    m_schedule.tv_nsec += T;
    normalize_schedule();
    rectify_schedule();
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &m_schedule, nullptr);
  }

private:
  constexpr static int64_t ONE_SEC_IN_NS = 1'000'000'000;
  struct timespec m_schedule;

  void rectify_schedule() {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    if ((now.tv_sec > m_schedule.tv_sec) ||
        ((now.tv_sec == m_schedule.tv_sec) && (now.tv_nsec >= m_schedule.tv_nsec))) {
      int64_t ns_diff =
          (now.tv_nsec - m_schedule.tv_nsec) + ONE_SEC_IN_NS * (now.tv_sec - m_schedule.tv_sec);
      if (ns_diff != 0) {
        int64_t overflow = ns_diff % T;
        m_schedule.tv_nsec += (T - overflow) + ns_diff;
      } else {
        m_schedule.tv_nsec += T;
      }
      normalize_schedule();
    } else {
      return;
    }
  }

  void normalize_schedule() {
    if (m_schedule.tv_nsec >= ONE_SEC_IN_NS) {
      m_schedule.tv_sec += m_schedule.tv_nsec / ONE_SEC_IN_NS;
      m_schedule.tv_nsec %= ONE_SEC_IN_NS;
    }
  }

  struct timespec get_schedule() { return m_schedule; }
}; // class Schedtimer

}; // namespace Asclepius
