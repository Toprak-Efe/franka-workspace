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
#include <tlib/control/devices.hpp>
#include <tlib/control/filters/clamping.hpp>
#include <tlib/control/signal.hpp>
#include <utility>

Asclepius::HapticDevice::HapticDevice(const std::string &id_name,
                                      SignalPort<Asclepius::TwistDisplacement, 1> *sensor_port,
                                      SignalPort<Wrench> *command_port)
    : DeviceInterface<Asclepius::TwistDisplacement, Wrench, 1>(sensor_port, command_port),
      differentiator_(1000.0), state_(State::Idle), flock_("asclepis", id_name) {
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
    : differentiator_{1000.0}, flock_(std::move(oth.flock_)),
      DeviceInterface(oth.sensor_, oth.command_), id_(oth.id_) {
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
    } else {
      hdWaitForCompletion(handle, HD_WAIT_INFINITE);
      if (HD_DEVICE_ERROR(error = hdGetError())) {
        ShutdownCoordinator::instance().shutdown_with_exception(std::make_exception_ptr(
            std::runtime_error("Unable to enter haptic loop, scheduler error.")));
      }
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

  const static Clamper<Wrench> force_limiter{
      Wrench(std::array<double, 6>{-4.0, -4.0, -4.0, -0.0, -0.0, -0.0}),
      Wrench(std::array<double, 6>{4.0, 4.0, 4.0, 0.0, 0.0, 0.0})};

  hdMakeCurrentDevice(haptic_device->id_);
  hdBeginFrame(haptic_device->id_);

  std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now();

  std::array<double, 16> T_raw{0.0};
  hdGetDoublev(HD_CURRENT_TRANSFORM, T_raw.data());
  Eigen::Map<const Eigen::Matrix4d> T(T_raw.data());

  Eigen::Vector3d position = T.block<3, 1>(0, 3);
  Eigen::AngleAxisd aa{T.block<3, 3>(0, 0)};
  Eigen::Vector3d angular_position = aa.angle() * aa.axis();
  Displacement displacement(position, angular_position, timestamp);

  Displacement diff_disp = haptic_device->differentiator_(displacement);
  diff_disp = haptic_device->moving_average_(diff_disp);
  Twist twist = static_cast<Twist>(diff_disp);
  twist.stamp() = timestamp;
  Asclepius::TwistDisplacement sensor{twist, displacement};
  haptic_device->sensor_->push(sensor);

  Wrench command = haptic_device->command_->sample();
  command = force_limiter.clamp(command);
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
