#pragma once

#include "franka.hpp"
#include "haptic.hpp"
#include <string>

namespace Asclepius {

template <bool TelemetryEnabled> class BilateralControl {
public:
  BilateralControl(const std::string &franka_host, const std::string &haptic_name)
      : m_franka_loop(franka_host), m_haptic_loop(haptic_name) {}
  ~BilateralControl() { stop(); }

  void start() {
    if (m_running().exchange(true))
        return;

    m_franka_loop.start();
    m_haptic_loop.start();
  }

  void stop() {
    m_franka_loop.stop();
    m_haptic_loop.stop();
  }

private:
  std::atomic_bool m_running();
  FrankaServoLoop<TelemetryEnabled> m_franka_loop;
  HapticServoLoop<TelemetryEnabled> m_haptic_loop;
  // No haptic thread - bad API
}; // class BilateralControl

}; // namespace Asclepius
