/*#pragma once

#include <datalog.hpp>
#include <Eigen/Geometry>
#include <HD/hd.h>
#include <array>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <data/cache.hpp>
#include <data/triplebuffer.hpp>
#include <franka/robot.h>
#include <memory>
#include <ranges>
#include <thread>

HDCallbackCode HDCALLBACK haptic_callback(void *data);

namespace Asclepius {

struct force_t {
  std::array<double, 6> data;
  std::chrono::time_point<std::chrono::steady_clock> timestamp;
};
struct velocity_t {
  std::array<double, 6> data;
  std::chrono::time_point<std::chrono::steady_clock> timestamp;
};
using force_buffer_t = TripleBuffer<force_t>;
using velocity_buffer_t = TripleBuffer<velocity_t>;

class FrankaInterface {
public:
  FrankaInterface(
      const std::string &,
      const std::shared_ptr<Timelog>);
  ~FrankaInterface();
  void start();
  void stop();

  force_buffer_t m_force;
  velocity_buffer_t m_velocity;

private:
  void _thread_function();
  std::shared_ptr<franka::Robot> m_robot;
  std::shared_ptr<std::thread> m_thread;
  std::shared_ptr<Timelog> m_datalog;
};

class HapticInterface {
public:
  HapticInterface(
      const std::string &,
      const std::shared_ptr<Timelog>);
  ~HapticInterface();
  void start();
  void stop();

  struct haptic_interface_buffer_t {
    force_buffer_t force;
    velocity_buffer_t velocity;
    HHD device_id;
    HDSchedulerHandle device_scheduler_id;
    std::shared_ptr<Timelog> datalog;
    haptic_interface_buffer_t() {};
  } m_buffer;

private:
  void _thread_function();
  std::shared_ptr<std::thread> m_thread;
};

class Pipeline {
public:
  Pipeline(
      const std::string &, const std::string &,
      const std::array<
          std::shared_ptr<Timelog>,
          3> &);
  ~Pipeline();
  void start();
  void stop();

private:
  void _thread_function();
  std::shared_ptr<Timelog> m_datalog;
  std::shared_ptr<std::thread> m_thread;
  Eigen::Quaterniond m_rotation_mapping;
  FrankaInterface m_franka_interface;
  HapticInterface m_haptic_interface;
};


}; // namespace Asclepius */
