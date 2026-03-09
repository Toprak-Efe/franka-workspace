#pragma once

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

using Timestamp = std::chrono::time_point<std::chrono::steady_clock>;
template <typename T>
concept Timestamped = requires(T a) {
  { a.timestamp } -> std::same_as<Timestamp &>;
  { a.data } -> std::ranges::range;
};

template <Timestamped T> class FirstOrderHold {
public:
  FirstOrderHold(double period) : m_period{period} {}
  void push(const T &sample) {
    m_samples[0] = m_samples[1];
    m_samples[1] = sample;
  }
  T sample() {
    Timestamp curr_time = std::chrono::steady_clock::now();
    Timestamp &prev_time = m_samples[1].timestamp;
    Timestamp &last_time = m_samples[0].timestamp;

    decltype(T::data) data = m_samples[0].data;
    for (auto &&[curr_sample, prev_sample, last_sample] :
         std::ranges::views::zip(data, m_samples[1].data, m_samples[0].data)) {
      curr_sample = (curr_time - prev_time) * (prev_sample - last_sample) /
                    (prev_time - last_time);
    }
    return {.data{data}, .timestamp{curr_time}};
  }

private:
  double m_period;
  std::array<T, 2> m_samples;
};

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
  std::shared_ptr<std::thread> m_thread;
  Eigen::Quaterniond m_rotation_mapping;
  void _thread_function();
  FrankaInterface m_franka_interface;
  HapticInterface m_haptic_interface;
};

class KilohertzLoop {
public:
  KilohertzLoop();
  void await();

private:
  static constexpr int64_t PERIOD_NS = 1000000;
  struct timespec m_schedule;
};

}; // namespace Asclepius
