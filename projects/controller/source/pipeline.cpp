#include "fmt/core.h"
#include <HD/hd.h>
#include <HD/hdDefines.h>
#include <HD/hdDevice.h>
#include <HDU/hduError.h>
#include <atomic>
#include <exception.hpp>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <functional>
#include <iostream>
#include <memory>
#include <pipeline.hpp>
#include <printf.h>
#include <shutdown.hpp>

HDCallbackCode HDCALLBACK haptic_callback(void *data) {
  if (Asclepius::g_shutdown_coordinator.is_shutdown()) {
    return HD_CALLBACK_DONE;
  }
  auto *buffer = (Asclepius::HapticInterface::haptic_interface_buffer_t *)data;
  hdMakeCurrentDevice(buffer->device_id);

  hdBeginFrame(buffer->device_id);
  /* Read Velocity */
  float velocity[6];
  hdGetFloatv(HD_CURRENT_VELOCITY, velocity);
  hdGetFloatv(HD_CURRENT_ANGULAR_VELOCITY, &velocity[3]);
  buffer->velocity.add({velocity[0], velocity[1], velocity[2], velocity[3],
                        velocity[4], velocity[5]});

  /* Write Force */
  std::array<double, 6> force_buff;
  if (buffer->force.get(force_buff)) {
    float force[6];
    for (size_t i = 0; i < 6; i++)
      force[i] = (float)force_buff[i];
    hdSetFloatv(HD_CURRENT_FORCE, force);
    hdSetFloatv(HD_CURRENT_TORQUE, &force[3]);
  }
  hdEndFrame(buffer->device_id);

  return HD_CALLBACK_CONTINUE;
}

Asclepius::FrankaInterface::FrankaInterface(const std::string &hostname) {
  try {
    m_robot = std::make_shared<franka::Robot>(hostname);
    m_robot->setCollisionBehavior({{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                                  {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                                  {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                                  {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                                  {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                                  {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
                                  {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
                                  {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
    m_robot->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
    m_robot->setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
  } catch (franka::NetworkException &e) {
    throw Asclepius::AsclepiusException(
        "Unable to connect to the Franka robot at: " + hostname);
  } catch (franka::IncompatibleVersionException &e) {
    throw Asclepius::AsclepiusException(
        "Franka at " + hostname +
        " has a mismatched firmware version with the compiled version of "
        "libfranka.");
  }
}

void Asclepius::FrankaInterface::start() {
  m_thread = std::make_shared<std::thread>(
      std::bind(&Asclepius::FrankaInterface::_thread_function, this));
}

void Asclepius::FrankaInterface::stop() { m_thread->join(); }

void Asclepius::FrankaInterface::_thread_function() {
  std::function<franka::CartesianVelocities(const franka::RobotState &,
                                            franka::Duration)>
      callback = [&](const franka::RobotState &state,
                     franka::Duration) -> franka::CartesianVelocities {
    if (Asclepius::g_shutdown_coordinator.is_shutdown()) {
      return franka::MotionFinished(franka::CartesianVelocities{0.0});
    }

    /* Read Force */
    std::array<double, 6> force = state.O_F_ext_hat_K;
    m_force.add(force);

    /* Write Velocity */
    static std::array<double, 6> velocity = {0.0};
    std::array<double, 6> velocity_new;
    if (m_velocity.get(velocity_new)) {
      velocity = velocity_new;
    }
    return velocity;
  };
}

Asclepius::HapticInterface::HapticInterface(const std::string &devicename) {
  HDErrorInfo error;
  m_buffer.device_id = hdInitDevice(devicename.c_str());
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Failed to initialize device.");
    throw AsclepiusException("Failed to initialize haptic device with name: " +
                             devicename);
  }
  hdEnable(HD_FORCE_OUTPUT);
}

void Asclepius::HapticInterface::start() {
  m_buffer.device_scheduler_id = hdScheduleAsynchronous(
      haptic_callback, (void *)&m_buffer, HD_DEFAULT_SCHEDULER_PRIORITY);
  m_thread = std::make_shared<std::thread>(std::bind(&Asclepius::HapticInterface::_thread_function, this));
}

void Asclepius::HapticInterface::_thread_function() {
  hdWaitForCompletion(m_buffer.device_scheduler_id, HD_WAIT_CHECK_STATUS);
  hdUnschedule(m_buffer.device_scheduler_id);
}

void Asclepius::HapticInterface::stop() { hdDisableDevice(m_buffer.device_id); }
