#pragma once

#include <HD/hdScheduler.h>
#include <atomic>
#include <fstream>
#include <franka/robot.h>
#include <thread>
#include <HD/hd.h>
#include "data/ringbuffer.hpp"
#include "data/triplebuffer.hpp"

namespace Asclepius {

struct HapticState {
    std::array<double, 16> transform;
    std::array<double, 3> linear_velocity;
    std::array<double, 3> angular_velocity;
    std::array<bool, 4> button_states;
}; // HapticState

struct HapticCommand {
    std::array<double, 6> forces;
}; // HapticCommand

struct RobotState {
  std::array<double, 16> transform;
  std::array<double, 7> joint_velocities;
  std::array<double, 7> joint_positions;
  std::array<double, 7> joint_contacts;
}; // struct RobotState 

struct RobotCommand {
    std::array<double, 3> pose;
    std::array<double, 3> orientation;
}; // struct RobotCommand

struct system_data_t {
    TripleBuffer<RobotState> robot_state;
    TripleBuffer<RobotCommand> robot_command;
    TripleBuffer<HapticState> haptic_state;
    TripleBuffer<HapticCommand> haptic_command;
}; // struct system_data_t

HDCallbackCode HDCALLBACK haptic_callback(void *data);

class App {
public:
    App(std::ofstream &m_log_file);
    App(App &&) = delete; 
    App(const App &) = delete; 
    App &operator=(App &&) = delete;
    App &operator=(const App &) = delete;
    ~App();

    void initialize();
private:
    void run();
    void _initialize_robot(size_t idx);
    void _initialize_haptic(size_t idx);
    void _initialize_controller();
    
    std::atomic_bool m_running;
    std::thread m_app_main_thread;

    std::array<std::shared_ptr<franka::Robot>, 2> m_robots;
    std::array<system_data_t, 2> system_data;

    std::ofstream &m_log_file; 
}; // class App

}; // namespace Asclepius

