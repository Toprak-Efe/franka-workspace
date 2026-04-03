#pragma once

#include <bilateralcontrol/common.hpp>
#include <tlib/concurrency/flock.hpp>
#include <tlib/control/devices.hpp>
#include <tlib/control/signal.hpp>
#include <tlib/control/spatial.hpp>

namespace Asclepius {

class FrankaDevice : DeviceInterface<WrenchDisplacement, Twist> {
public:
  FrankaDevice(const std::string &device_name, const std::string &host_name,
               SignalPort<WrenchDisplacement> *sensor_port, SignalPort<Twist> *command_port);
  FrankaDevice(FrankaDevice &&);
  FrankaDevice(const FrankaDevice &) = delete;
  ~FrankaDevice();

  void start() override;
  void stop() override;

private:
  std::jthread worker_;
  enum class State : uint8_t { Idle, Running, Halting };
  std::atomic<State> state_;
  FileLock<std::string> flock_;
  std::string host_name_;
}; // class FrankaDevice

}; // namespace Asclepius
