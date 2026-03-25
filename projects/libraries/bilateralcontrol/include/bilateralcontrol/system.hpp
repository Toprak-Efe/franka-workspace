#pragma once

#include "bilateralcontrol/control.hpp"
#include "franka.hpp"
#include "haptic.hpp"
#include <string>

namespace Asclepius {

/**
 * @note The methods of this class are NOT thread-safe.
 */
template <bool TelemetryEnabled> class System {
public:
  System(const std::string &franka_host, const std::string &haptic_name)
      : m_franka_loop(franka_host), m_haptic_loop(haptic_name) {}
  ~System() { stop(); }

  void start() {
    m_main_loop.start(m_franka_loop.getBuffer(), m_haptic_loop.getBuffer());
    m_franka_loop.start();
    m_haptic_loop.start();
  }

  void stop() {
    m_main_loop.stop();
    m_franka_loop.stop();
    m_haptic_loop.stop();
  }

private:
  FrankaServoLoop<TelemetryEnabled> m_franka_loop;
  HapticServoLoop<TelemetryEnabled> m_haptic_loop;
  MainControlLoop<TelemetryEnabled> m_main_loop;
}; // class System

}; // namespace Asclepius
