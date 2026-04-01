#pragma once

#include <HD/hdScheduler.h>
#include <bilateralcontrol/common.hpp>
#include <bilateralcontrol/shutdown.hpp>
#include <cstdint>
#include <string>
#include <thread>
#include <tlib/concurrency/flock.hpp>
#include <tlib/control/devices.hpp>
#include <tlib/control/signal.hpp>
#include <tlib/control/spatial.hpp>

namespace Asclepius {

class HapticDevice : DeviceInterface<TwistDisplacement, Wrench> {
public:
  HapticDevice() = delete;
  explicit HapticDevice(const std::string &device_name, SignalPort<TwistDisplacement> *sensor_port,
                        SignalPort<Wrench> *command_port);
  HapticDevice(HapticDevice &&); 
  HapticDevice(const HapticDevice &) = delete;
  ~HapticDevice();

  void start() override;
  void stop() override;

private:
  HHD id_;
  static HDCallbackCode io_callback(void *);
  static class HapticSchedulerCounter {
  public:
    void increment();
    void decrement();

  private:
    std::atomic_uint32_t schedule_count_{0};
  } scheduler_counter_;

private:
  std::jthread worker_;
  enum class State : uint8_t { Idle, Running, Halting };
  std::atomic<State> state_;
  FileLock<std::string> flock_;
}; // class HapticDevice

}; // namespace Asclepius
