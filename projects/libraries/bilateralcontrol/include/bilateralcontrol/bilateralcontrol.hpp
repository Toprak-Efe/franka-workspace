#pragma once

#include "franka.hpp"
#include "haptic.hpp"
#include <mutex>
#include <string>

namespace Asclepius {

/**
 * @note The methods of this class are NOT thread-safe.
 */
template <bool TelemetryEnabled> class BilateralControl {
public:
  BilateralControl(const std::string &franka_host, const std::string &haptic_name)
      : m_running{false}, m_franka_loop(franka_host), m_haptic_loop(haptic_name) {}
  ~BilateralControl() { stop(); }

  void start() {
    m_franka_loop.start();
    m_haptic_loop.start();
  }

  void stop() {
    m_franka_loop.stop();
    m_haptic_loop.stop();
  }

private:
  bool m_running;
  std::mutex m_lock;
  FrankaServoLoop<TelemetryEnabled> m_franka_loop;
  HapticServoLoop<TelemetryEnabled> m_haptic_loop;
}; // class BilateralControl

}; // namespace Asclepius
