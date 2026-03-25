#pragma once

#include <HD/hd.h>
#include <HD/hdDefines.h>
#include <HD/hdDevice.h>
#include <HD/hdScheduler.h>
#include <HDU/hduError.h>
#include <bilateralcontrol/common.hpp>
#include <exception>
#include <mutex>
#include <snippets/control/nthhold.hpp>
#include <sstream>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>

HDCallbackCode HDCALLBACK asclepius_haptic_callback(void *buffer);

namespace Asclepius {

struct HapticBuffer {
  std::stop_token *p_stop_token;
  HHD device_handle;
  HDErrorInfo error;
  ZeroOrderHold<Wrench> zoh;
  WrenchTwistDisplacementBuffer buffer;
}; // struct HapticBuffer

class HapticSchedulerCounter {
public:
  void incremenent();
  void decrement();

private:
  std::mutex m_lock;
  uint32_t m_scheduled{0};
}; // HapticSchedulerCounter

template <bool TelemetryEnabled> class HapticServoLoop {
public:
  HapticServoLoop(const std::string &devicename) : m_device_name{devicename} {}
  ~HapticServoLoop() { stop(); }

  template <typename T>
  requires requires(T &c, std::exception_ptr &e) {
    c.shutdown();
    c.shutdown_exception(e);
  }
  void start(T &shutdown_coordinator) {
    std::lock_guard<std::mutex> guard{m_lock};
    if (m_running) {
      return;
    } else {
      m_running = true;
    }

    m_thread = std::jthread([&](std::stop_token stoken) {
      HDErrorInfo error;
      m_haptic_buffer.device_handle = hdInitDevice(m_device_name.c_str());
      if (HD_DEVICE_ERROR(error = hdGetError())) {
        std::stringstream errstream{};
        errstream << error;
        try {
          throw std::runtime_error{errstream.str()};
        } catch (...) {
          shutdown_coordinator.shutdown_exception(std::current_exception());
          return;
        }
      }

      hdMakeCurrentDevice(m_haptic_buffer.device_handle);
      hdEnable(HD_FORCE_OUTPUT);

      s_haptic_counter.incremenent();
      m_haptic_buffer.p_stop_token = &stoken;
      hdScheduleSynchronous(&asclepius_haptic_callback, &m_haptic_buffer,
                            HD_DEFAULT_SCHEDULER_PRIORITY);

      if (HD_DEVICE_ERROR(m_haptic_buffer.error)) {
        std::stringstream errstream{};
        errstream << error;
        try {
          throw std::runtime_error{errstream.str()};
        } catch (...) {
          shutdown_coordinator.shutdown_exception(std::current_exception());
        }
      }

      s_haptic_counter.decrement();
      hdDisableDevice(m_haptic_buffer.device_handle);
      std::lock_guard<std::mutex> guard{m_lock};
      m_running = false;
      m_haptic_buffer.device_handle = 0;
      m_haptic_buffer.error.errorCode = 0;
      m_haptic_buffer.buffer.tdbuff.reset();
      m_haptic_buffer.buffer.wbuff.reset();
    });
  }

  void stop() {
    std::jthread local_thread;
    {
      std::lock_guard<std::mutex> guard{m_lock};
      std::swap(m_thread, local_thread);
    }
    local_thread.request_stop();
    if (local_thread.joinable()) {
      local_thread.join();
    }
  }

  WrenchTwistDisplacementBuffer &getBuffer() { return m_haptic_buffer.buffer; }

private:
  std::mutex m_lock;
  bool m_running{false};
  std::jthread m_thread;
  std::string m_device_name;
  HapticBuffer m_haptic_buffer{};
  static HapticSchedulerCounter s_haptic_counter;
}; // class HapticServoLoop;

}; // namespace Asclepius
