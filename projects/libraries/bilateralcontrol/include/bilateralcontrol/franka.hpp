#pragma once

#include <bilateralcontrol/common.hpp>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <exception>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <mutex>
#include <snippets/concepts/vector.hpp>
#include <snippets/control/nthhold.hpp>
#include <string>
#include <thread>

namespace Asclepius {

static constexpr double FrankaMaximumLinearSpeed = 0.08; // 8cm/s
static constexpr double FrankaMaximumAngularSpeed = 0.08; // 8cm/s

template <bool TelemetryEnabled> class FrankaServoLoop {
public:
  FrankaServoLoop(const std::string &hostname) : m_franka_hostname(hostname) {
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

        ZeroOrderHold<Twist> twist_hold;
        franka.control([&](const franka::RobotState &state,
                           franka::Duration duration) -> franka::CartesianVelocities {
          Eigen::Matrix4<double> o_t_ee =
              Eigen::Map<const Eigen::Matrix4<double>>(state.O_T_EE.data());
          Eigen::Vector3<double> origin;
          origin.setZero();
          Eigen::Vector3<double> end_effector = o_t_ee * origin;
          WrenchDisplacement wd{.wrench{state.O_F_ext_hat_K}, .displacement{end_effector}};
          m_buffer.wdbuff.add(wd);

          Twist t;
          if (m_buffer.tbuff.get(t)) {
            twist_hold.push(t);
          }
          t = twist_hold.sample();
          std::array<double, 6> cmd_arr;
          Eigen::Map<Eigen::Vector<double, 6>>(cmd_arr.data()) = t.vec();
          franka::CartesianVelocities cmd(cmd_arr);

          if (stoken.stop_requested()) {
            return franka::MotionFinished(cmd);
          } else {
            return cmd;
          }
        });
      } catch (...) {
        std::exception_ptr ex = std::current_exception();
        shutdown_coordinator.shutdown_exception(ex);
      }
      std::lock_guard<std::mutex> lock(m_lock);
      m_running = false;
      m_buffer.tbuff.reset();
      m_buffer.wdbuff.reset();
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

  TwistWrenchDisplacementBuffer &getBuffer() { return m_buffer; }

private:
  std::mutex m_lock;
  bool m_running{false};
  std::jthread m_thread;
  std::string m_franka_hostname;
  TwistWrenchDisplacementBuffer m_buffer;
}; // class Asclepius

}; // namespace Asclepius
