#pragma once

#include <HD/hd.h>
#include <array>
#include <data/cache.hpp>
#include <data/triplebuffer.hpp>
#include <franka/robot.h>
#include <memory>
#include <thread>

namespace Asclepius {

HDCallbackCode HDCALLBACK haptic_callback(void *data);

using force_t = std::array<double, 6>;
using velocity_t = std::array<double, 6>;
using force_buffer_t = TripleBuffer<force_t>;
using velocity_buffer_t = TripleBuffer<velocity_t>;

class FrankaInterface {
public:
  FrankaInterface(const std::string &);
  void start();
  void stop();

  force_buffer_t m_force;
  velocity_buffer_t m_velocity;

private:
  void _thread_function();
  std::shared_ptr<franka::Robot> m_robot;
  std::shared_ptr<std::thread> m_thread;
};

class HapticInterface {
public:
  HapticInterface(const std::string &);
  void start();
  void stop();

  struct haptic_interface_buffer_t {
    force_buffer_t force;
    velocity_buffer_t velocity;
    HHD device_id;
    HDSchedulerHandle device_scheduler_id;
  } m_buffer;

private:
  void _thread_function();
  std::shared_ptr<std::thread> m_thread;
};

class Pipeline {
public:
  Pipeline(const std::string &, const std::string &);
  void start();
  void stop();

private:
  FrankaInterface m_franka_interface;
  HapticInterface m_haptic_interface;
};

}; // namespace Asclepius
