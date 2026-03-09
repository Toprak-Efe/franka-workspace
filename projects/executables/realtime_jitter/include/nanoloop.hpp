#pragma once

#include <ctime>
#include <stdint.h>

template <int64_t PERIOD> class Nanoloop {
public:
  Nanoloop() { clock_gettime(CLOCK_MONOTONIC, &m_schedule); }

  void await() {
    m_schedule.tv_nsec += PERIOD;
    while (m_schedule.tv_nsec >= 1000000000) {
      m_schedule.tv_sec += 1;
      m_schedule.tv_nsec -= 1000000000;
    }

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (now.tv_sec > m_schedule.tv_sec ||
        ((now.tv_sec == m_schedule.tv_sec) &&
         (now.tv_nsec >= m_schedule.tv_nsec))) {
      while (now.tv_sec > m_schedule.tv_sec ||
             ((now.tv_sec == m_schedule.tv_sec) &&
              (now.tv_nsec >= m_schedule.tv_nsec))) {
        m_schedule.tv_nsec += PERIOD;
        while (m_schedule.tv_nsec >= 1000000000) {
          m_schedule.tv_sec += 1;
          m_schedule.tv_nsec -= 1000000000;
        }
      }
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &m_schedule, nullptr);
  }

private:
  struct timespec m_schedule;
};
