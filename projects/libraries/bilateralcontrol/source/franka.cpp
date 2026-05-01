#include "bilateralcontrol/shutdown.hpp"
#include "tlib/control/devices.hpp"
#include "tlib/control/filters/clamping.hpp"
#include "tlib/control/spatial.hpp"
#include <bilateralcontrol/common.hpp>
#include <bilateralcontrol/franka.hpp>
#include <chrono>
#include <exception>
#include <franka/control_types.h>
#include <franka/robot.h>
#include <stdexcept>
#include <stop_token>
#include <utility>

Asclepius::FrankaDevice::FrankaDevice(const std::string &device_name, const std::string &host_name,
                                      SignalPort<Asclepius::WrenchDisplacement> *sensor_port,
                                      SignalPort<Twist> *command_port)
    : DeviceInterface<Asclepius::WrenchDisplacement, Twist>(sensor_port, command_port),
      host_name_(host_name), state_(State::Idle), flock_("asclepius", device_name),
      differentiator_() {}

Asclepius::FrankaDevice::FrankaDevice(FrankaDevice &&oth)
    : flock_(std::move(oth.flock_)), DeviceInterface(oth.sensor_, oth.command_),
      host_name_(oth.host_name_), differentiator_()

{
  state_ = State::Idle;
  oth.stop();
  oth.command_ = nullptr;
  oth.sensor_ = nullptr;
}

Asclepius::FrankaDevice::~FrankaDevice() {
  auto expected = State::Running;
  if (!state_.compare_exchange_strong(expected, State::Halting)) {
    return;
  }

  worker_.request_stop();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void Asclepius::FrankaDevice::start() {
  auto expected = State::Idle;
  if (!state_.compare_exchange_strong(expected, State::Running)) {
    throw std::logic_error{"Franka: cannot run when not idle."};
  }

  worker_ = std::jthread([&](std::stop_token st) {
    try {
      const static Clamper<Twist> velocity_limiter{
          Twist(std::array<double, 6>{-0.02, -0.02, -0.02, -0.02, -0.02, -0.02}),
          Twist(std::array<double, 6>{0.02, 0.02, 0.02, 0.02, 0.02, 0.02})};

      franka::Robot robot{host_name_};
      robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
      std::array<double, 7> lower_torque_thresholds_nominal{
          {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
      std::array<double, 7> upper_torque_thresholds_nominal{
          {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
      std::array<double, 7> lower_torque_thresholds_acceleration{
          {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
      std::array<double, 7> upper_torque_thresholds_acceleration{
          {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
      std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
      std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
      std::array<double, 6> lower_force_thresholds_acceleration{
          {30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
      std::array<double, 6> upper_force_thresholds_acceleration{
          {40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
      robot.setCollisionBehavior(
          lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
          lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
          lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
          lower_force_thresholds_nominal, upper_force_thresholds_nominal);
      robot.control([&](const franka::RobotState &state,
                        franka::Duration duration) -> franka::CartesianVelocities {
        auto stamp = std::chrono::steady_clock::now();
        Eigen::Map<const Eigen::Matrix4d> T(state.O_T_EE.data());

        Eigen::Vector3d position = T.block<3, 1>(0, 3);
        Eigen::AngleAxisd aa{T.block<3, 3>(0, 0)};
        Eigen::Vector3d angular_position = aa.angle() * aa.axis();
        Displacement displacement{position, angular_position, stamp};
        Wrench wrench(state.O_F_ext_hat_K, stamp);
        WrenchDisplacement wd{wrench, displacement};
        sensor_->push(wd);

        std::array<double, 6> command;
        Eigen::Map<Twist::Vector6>(command.data()) = command_->sample().vec();

        if (st.stop_requested()) {
          return franka::MotionFinished(
              franka::CartesianVelocities({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
        } else {
          return franka::CartesianVelocities(command);
        }
      });
      auto expected = State::Halting;
      if (!state_.compare_exchange_strong(expected, State::Idle)) {
        throw std::logic_error{"Franka: cannot idle when not stopping."};
      }
    } catch (...) {
      ShutdownCoordinator::instance().shutdown_with_exception(
          std::make_exception_ptr(std::current_exception()));
    }
  });
}

void Asclepius::FrankaDevice::stop() {
  auto expected = State::Running;
  if (!state_.compare_exchange_strong(expected, State::Halting)) {
    throw std::logic_error("Franka: cannot stop when not running.");
  }

  worker_.request_stop();
  if (worker_.joinable()) {
    worker_.join();
  }
}
