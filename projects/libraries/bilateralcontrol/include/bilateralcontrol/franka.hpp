#pragma once

#include <bilateralcontrol/servo.hpp>
#include <chrono>
#include <exception>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <memory>
#include <mutex>
#include <thread>

namespace Asclepius {

template <bool TelemetryEnabled> class FrankaServoLoop {
public:
  FrankaServoLoop(const std::string &hostname, std::shared_ptr<ForceVelocityBuffer> &buffer)
      : m_franka_hostname(hostname), m_buffer(buffer) {
    return;
  }
  ~FrankaServoLoop() { stop(); }

  template <typename T>
  requires requires(T &c) {
    { c.shutdown() };
    { c.shutdown_exception() };
  }
  void start(T &shutdown_coordinator) {
    std::lock_guard<std::mutex> lock_guard(m_lock);
    if (m_running) {
      return;
    } else {
      m_running = true;
    }

    m_thread = std::jthread{[&](std::stop_token stoken) {
      try {
        franka::Robot franka(m_franka_hostname);
        franka.setCollisionBehavior(
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
            {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
        franka.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        franka.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

        Velocities vw_prev{.data{0}, .timestamp{}};
        franka.control([&](const franka::RobotState &state,
                           franka::Duration duration) -> franka::CartesianVelocities {
          Forces ft{.data{state.O_F_ext_hat_K}, .timestamp{std::chrono::steady_clock::now()}};
          m_buffer->force.add(ft);

          Velocities vw;
          if (m_buffer->velocity.get(vw))
            vw_prev = vw;
          else
            vw = vw_prev;
          if (stoken.stop_requested()) {
            return franka::MotionFinished(vw.data);
          } else
            return vw.data;
        });
      } catch (...) {
        std::exception_ptr ex = std::current_exception();
        shutdown_coordinator.shutdown_exception(ex);
      }
      std::lock_guard<std::mutex> lock(m_lock);
      m_running = false;
      m_buffer->force.reset();
      m_buffer->velocity.reset();
    }};
  }

  void stop() {
    std::jthread local_thread;
    {
      std::lock_guard<std::mutex> lock(m_lock);
      std::swap(local_thread, m_thread);
    }
    local_thread.request_stop();
    if (local_thread.joinable()) {
      local_thread.join();
    }
  }

private:
  std::mutex m_lock;
  std::jthread m_thread;
  bool m_running{false};
  std::string m_franka_hostname;
  std::shared_ptr<ForceVelocityBuffer> m_buffer;
}; // class Asclepius

}; // namespace Asclepius
