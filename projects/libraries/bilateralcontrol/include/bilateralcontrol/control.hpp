#pragma once

#include "realtimeloop/monoloop.hpp"
#include <eigen3/Eigen/Eigen>
#include <mutex>
#include <snippets/control/physics.hpp>
#include <thread>
#include <utility>

namespace Asclepius {

template <bool TelemetryEnabled> class MainControlLoop {
public:
  MainControlLoop(double m_v = 1.0, double k_v = 1.0, double b_v = 1.0, double a = 1.0)
      : m_v{m_v}, k_v{k_v}, b_v{b_v}, a{a} {}
  ~MainControlLoop() { stop(); }

  void start(ForceVelocityBuffer &m_buff, ForceVelocityBuffer &s_buff) {
    std::lock_guard<std::mutex> guard{m_lock};
    if (m_running) {
      return;
    } else {
      m_running = true;
    }

    m_thread = std::jthread{[&](std::stop_token stoken) {
      Monoloop<void(), 1'000'000, TelemetryEnabled> loop{stoken, [&]() {

                                                         }};
      m_running = false;
    }};
  }

  void stop() {
    std::jthread local_thread;
    {
      std::lock_guard<std::mutex> guard{m_lock};
      std::swap(local_thread, m_thread);
    }
    local_thread.request_stop();
    if (local_thread.joinable()) {
      local_thread.join();
    }
  }

private:
  double m_v, k_v, b_v, a;
  std::mutex m_lock;
  std::jthread m_thread;
  bool m_running{false};
}; // class MainControlLoop

}; // namespace Asclepius
