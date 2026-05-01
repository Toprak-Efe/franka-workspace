#pragma once

#include <bilateralcontrol/franka.hpp>
#include <bilateralcontrol/haptic.hpp>
#include <thread>
#include <tlib/control/signal.hpp>

class Pipeline {
public:
  Pipeline() = delete;
  Pipeline(SignalPort<Asclepius::TwistDisplacement> *haptic_sensor_port,
           SignalPort<Wrench> *haptic_command_port,
           SignalPort<Asclepius::WrenchDisplacement> *franka_sensor_port,
           SignalPort<Twist> *franka_command_port);
  ~Pipeline();

  void start();
  void stop();

private:

private:
  std::jthread worker_;
  enum class State : uint8_t { Idle, Running, Halting };
  std::atomic<State> state_;
};
