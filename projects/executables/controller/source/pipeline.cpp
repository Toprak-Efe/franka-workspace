#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/src/Geometry/Quaternion.h>
#include <HD/hd.h>
#include <HD/hdDefines.h>
#include <HD/hdDevice.h>
#include <HD/hdScheduler.h>
#include <HDU/hduError.h>
#include <chrono>
#include <ctime>
#include <exception.hpp>
#include <filter.hpp>
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
#include <pthread.h>
#include <sched.h>
#include <shutdown.hpp>

HDCallbackCode HDCALLBACK haptic_callback(void *data) {
  if (Asclepius::g_shutdown_coordinator.is_shutdown()) {
    return HD_CALLBACK_DONE;
  }
  auto *buffer = (Asclepius::HapticInterface::haptic_interface_buffer_t *)data;
  hdMakeCurrentDevice(buffer->device_id);

  hdBeginFrame(buffer->device_id);
  /* Read Velocity */
  Asclepius::velocity_t velocity;
  hdGetFloatv(HD_CURRENT_VELOCITY, (HDfloat *)&velocity.data[0]);
  hdGetFloatv(HD_CURRENT_ANGULAR_VELOCITY, (HDfloat *)&velocity.data[3]);
  velocity.timestamp = std::chrono::steady_clock::now();
  buffer->velocity.add(velocity);

  /* Write Force */
  Asclepius::force_t force_buff;
  if (buffer->force.get(force_buff)) {
    float force[6];
    for (size_t i = 0; i < 6; i++)
      force[i] = (float)force_buff.data[i];
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
    m_force.add({.data{state.O_F_ext_hat_K},
                 .timestamp{std::chrono::steady_clock::now()}});

    /* Write Velocity */
    static std::array<double, 6> velocity = {0.0};
    velocity_t velocity_new;
    if (m_velocity.get(velocity_new)) {
      velocity = velocity_new.data;
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
  m_thread = std::make_shared<std::thread>(
      std::bind(&Asclepius::HapticInterface::_thread_function, this));
}

void Asclepius::HapticInterface::_thread_function() {
  hdWaitForCompletion(m_buffer.device_scheduler_id, HD_WAIT_CHECK_STATUS);
}

void Asclepius::HapticInterface::stop() {
  hdUnschedule(m_buffer.device_scheduler_id);
  hdDisableDevice(m_buffer.device_id);
}

Asclepius::Pipeline::Pipeline(const std::string &hostname,
                              const std::string &devicename)
    : m_franka_interface(hostname), m_haptic_interface(devicename),
      m_rotation_mapping(Eigen::Quaterniond::Identity()) {}

void Asclepius::Pipeline::start() {
  m_haptic_interface.start();
  m_franka_interface.start();
  m_thread = std::make_shared<std::thread>(
      std::bind(&Asclepius::Pipeline::_thread_function, this));
}

void Asclepius::Pipeline::stop() {
  m_franka_interface.stop();
  m_haptic_interface.stop();
  m_thread->join();
}

void Asclepius::Pipeline::_thread_function() {
  int sched_priority = sched_get_priority_max(SCHED_FIFO);
  if (sched_priority < 0) {
    g_shutdown_coordinator.shutdown();
    std::cerr << "Unable to get the maximum schedule priority parameter.\n";
    return;
  }

  sched_param sched_parameter{sched_priority};
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sched_parameter) < 0) {
    g_shutdown_coordinator.shutdown();
    std::cerr << "Unable to get the maximum schedule priority.\n";
    return;
  }

  std::array<impedance_filter_t, 6> master_force_filters;
  for (auto &filter : master_force_filters) {
    init_impedance_filter(&filter, 1.0, 1.0, 1e-6);
  }
  FirstOrderHold<Asclepius::velocity_t> master_velocity_foh{1e-6};
  force_t slave_force_zoh;

  KilohertzLoop loop_timer;
  while (!g_shutdown_coordinator.is_shutdown()) {
    force_t slave_force;
    velocity_t master_velocity;
    if (m_franka_interface.m_force.get(slave_force)) {
      slave_force_zoh = slave_force;
    } else {
      slave_force = slave_force_zoh;
    }

    if (m_haptic_interface.m_buffer.velocity.get(master_velocity)) {
      master_velocity_foh.push(master_velocity);
    } else {
      master_velocity = master_velocity_foh.sample();
    }

    auto time = std::chrono::steady_clock::now();
    force_t master_force{.data{0.0}, .timestamp{time}};
    velocity_t slave_velocity{.data{0.0}, .timestamp{time}};

    Eigen::Vector3d e_sfr{slave_force.data[0], slave_force.data[1],
                          slave_force.data[2]};
    Eigen::Vector3d e_str{slave_force.data[3], slave_force.data[4],
                          slave_force.data[5]};
    e_sfr = m_rotation_mapping * e_sfr;
    e_str = m_rotation_mapping * e_str;
    Eigen::Vector3d e_mvr{master_velocity.data[0], master_velocity.data[1],
                          master_velocity.data[2]};
    Eigen::Vector3d e_mwr{master_velocity.data[3], master_velocity.data[4],
                          master_velocity.data[5]};
    e_mvr = m_rotation_mapping.inverse() * e_mvr;
    e_mwr = m_rotation_mapping.inverse() * e_mwr;
    std::array<double, 6> master_velocity_rotated{e_mvr[0], e_mvr[1], e_mvr[2],
                                                  e_mwr[0], e_mwr[1], e_mwr[2]};
    std::array<double, 6> slave_force_rotated{e_sfr[0], e_sfr[1], e_sfr[2],
                                              e_str[0], e_str[1], e_str[2]};

    for (size_t i = 0; i < 6; i++) {
      master_force.data[i] = sample_impedance_filter(&master_force_filters[i],
                                                     master_velocity.data[i],
                                                     slave_force_rotated[i]);
      slave_velocity.data[i] = master_velocity_rotated[i];
    }

    m_franka_interface.m_velocity.add(slave_velocity);
    m_haptic_interface.m_buffer.force.add(master_force);

    loop_timer.await();
  }
}

Asclepius::KilohertzLoop::KilohertzLoop() {
  clock_gettime(CLOCK_MONOTONIC, &m_schedule);
}

void Asclepius::KilohertzLoop::await() {
  m_schedule.tv_nsec += PERIOD_NS;
  while (m_schedule.tv_nsec >= 1000000000) {
    m_schedule.tv_sec += 1;
    m_schedule.tv_nsec -= 1000000000;
  }

  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  if (now.tv_sec > m_schedule.tv_sec || ((now.tv_sec == m_schedule.tv_sec) &&
                                         (now.tv_nsec >= m_schedule.tv_nsec))) {
    while (now.tv_sec > m_schedule.tv_sec ||
           ((now.tv_sec == m_schedule.tv_sec) &&
            (now.tv_nsec >= m_schedule.tv_nsec))) {
      m_schedule.tv_nsec += PERIOD_NS;
      while (m_schedule.tv_nsec >= 1000000000) {
        m_schedule.tv_sec += 1;
        m_schedule.tv_nsec -= 1000000000;
      }
    }
  }

  clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &m_schedule, nullptr);
}
