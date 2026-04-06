#include <HD/hdDefines.h>
#include <HD/hdDevice.h>
#include <HD/hdScheduler.h>
#include <bilateralcontrol/common.hpp>
#include <bilateralcontrol/haptic.hpp>
#include <bilateralcontrol/shutdown.hpp>
#include <chrono>
#include <eigen3/Eigen/Geometry>
#include <exception>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <tlib/control/calculus.hpp>
#include <tlib/control/devices.hpp>
#include <tlib/control/signal.hpp>
#include <utility>

Asclepius::HapticDevice::HapticDevice(const std::string &id_name,
                                      SignalPort<Asclepius::TwistDisplacement> *sensor_port,
                                      SignalPort<Wrench> *command_port)
    : DeviceInterface<Asclepius::TwistDisplacement, Wrench>(sensor_port, command_port),
      low_pass_(0.00782021, 0.01564042, 0.00782021, -1.73472577, 0.7660066), state_(State::Idle),
      flock_("asclepis", id_name), debug_("transform"){
  HDErrorInfo error;
  id_ = hdInitDevice(id_name.c_str());
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    auto exception =
        std::runtime_error(std::format("Unable to initialize haptic device: {}.", id_name.c_str()));
    ShutdownCoordinator::instance().shutdown_with_exception(std::make_exception_ptr(exception));
    throw exception;
  }
}

Asclepius::HapticDevice::HapticDevice(HapticDevice &&oth)
    : flock_(std::move(oth.flock_)), DeviceInterface(oth.sensor_, oth.command_), id_(oth.id_), debug_("transform") {
  state_ = State::Idle;
  oth.stop();
  oth.id_ = 0;
  oth.sensor_ = nullptr;
  oth.command_ = nullptr;
}

Asclepius::HapticDevice::~HapticDevice() {
  auto expected = State::Running;
  if (!state_.compare_exchange_strong(expected, State::Halting)) {
    return;
  }

  worker_.request_stop();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void Asclepius::HapticDevice::start() {
  auto expected = State::Idle;
  if (!state_.compare_exchange_strong(expected, State::Running)) {
    throw std::logic_error{"Haptic: cannot run when not idle."};
  }

  worker_ = std::jthread{[this](std::stop_token stoken) {
    hdMakeCurrentDevice(id_);
    hdEnable(HD_FORCE_OUTPUT);

    scheduler_counter_.increment();
    HDSchedulerHandle handle = hdScheduleAsynchronous(Asclepius::HapticDevice::io_callback, this,
                                                      HD_DEFAULT_SCHEDULER_PRIORITY);
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError())) {
      ShutdownCoordinator::instance().shutdown_with_exception(
          std::make_exception_ptr(std::runtime_error("Failed to schedule async haptic callback.")));
    }
    hdWaitForCompletion(handle, HD_WAIT_INFINITE);

    if (HD_DEVICE_ERROR(error = hdGetError())) {
      ShutdownCoordinator::instance().shutdown_with_exception(std::make_exception_ptr(
          std::runtime_error("Unable to enter haptic loop, scheduler error.")));
      // Not returning immediately, as there is a chance the device could be running.
      // Better to attempt cleanup.
    }
    scheduler_counter_.decrement();

    hdDisable(HD_FORCE_OUTPUT);
    hdDisableDevice(id_);

    auto expected = State::Halting;
    if (!state_.compare_exchange_strong(expected, State::Idle)) {
      ShutdownCoordinator::instance().shutdown_with_exception(
          std::make_exception_ptr(std::logic_error("Haptic: cannot idle when not halting.")));
    }
  }};
}

void Asclepius::HapticDevice::stop() {
  auto expected = State::Running;
  if (!state_.compare_exchange_strong(expected, State::Halting)) {
    ShutdownCoordinator::instance().shutdown_with_exception(
        std::make_exception_ptr(std::logic_error("Haptic: cannot halt when not running.")));
    return;
  }

  worker_.request_stop();
  if (worker_.joinable()) {
    worker_.join();
  }
}

HDCallbackCode Asclepius::HapticDevice::io_callback(void *ptr) {
  using namespace Asclepius;
  HapticDevice *haptic_device = static_cast<HapticDevice *>(ptr);

  hdMakeCurrentDevice(haptic_device->id_);
  hdBeginFrame(haptic_device->id_);

  std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now();
  Eigen::Vector3d position, angular_position;
  Displacement displacement;
  Twist twist;

  std::array<double, 16> T_raw;
  haptic_device->debug_.push(T_raw);
  hdGetDoublev(HD_CURRENT_TRANSFORM, T_raw.data());
  Eigen::Map<Eigen::Matrix4d> T(T_raw.data());
  position = T.block<3, 1>(0, 3);
  Eigen::AngleAxisd aa{T.block<3, 3>(0, 0)};
  angular_position = aa.angle() * aa.axis();
  twist = static_cast<Twist>(haptic_device->differentiator_(displacement));
  twist = haptic_device->low_pass_.sample(twist);
  twist.stamp() = timestamp;
  Asclepius::TwistDisplacement sensor{twist, Displacement{position, angular_position, timestamp}};
  haptic_device->sensor_->push(sensor);

  Wrench command = haptic_device->command_->sample();
  hdSetDoublev(HD_CURRENT_FORCE, command.linear().data());
  hdSetDoublev(HD_CURRENT_GIMBAL_TORQUE, command.angular().data());

  hdEndFrame(haptic_device->id_);

  return haptic_device->state_.load() == State::Halting ? HD_CALLBACK_DONE : HD_CALLBACK_CONTINUE;
}

Asclepius::HapticDevice::HapticSchedulerCounter Asclepius::HapticDevice::scheduler_counter_{};

void Asclepius::HapticDevice::HapticSchedulerCounter::increment() {
  if (schedule_count_.fetch_add(1) == 0) {
    hdStartScheduler();
  }
}

void Asclepius::HapticDevice::HapticSchedulerCounter::decrement() {
  uint32_t expected = 1;
  if (schedule_count_.compare_exchange_strong(expected, 0)) {
    hdStopScheduler();
  }
}
