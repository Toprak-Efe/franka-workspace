/*#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/src/Geometry/Quaternion.h>
#include <HD/hd.h>
#include <HD/hdDefines.h>
#include <HD/hdDevice.h>
#include <HD/hdScheduler.h>
#include <HDU/hduError.h>
#include <chrono>
#include <ctime>
#include <datalog.hpp>
#include <exception.hpp>
#include <filter.hpp>
#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <functional>
#include <iostream>
#include <iterator>
#include <memory>
#include <pipeline.hpp>
#include <printf.h>
#include <pthread.h>
#include <sched.h>
#include <shutdown.hpp>

HDCallbackCode HDCALLBACK haptic_callback(void *data) {
  Asclepius::ThreadLoopMeasure loop_measurement{
      .begin{std::chrono::steady_clock::now()}, .end{}};
  auto *buffer = (Asclepius::HapticInterface::haptic_interface_buffer_t *)data;
  if (Asclepius::g_shutdown_coordinator.is_shutdown()) {
    loop_measurement.end = std::chrono::steady_clock::now();
    buffer->datalog->push(loop_measurement);
    return HD_CALLBACK_DONE;
  }
  hdMakeCurrentDevice(buffer->device_id);

  hdBeginFrame(buffer->device_id);
  // Read Velocity 
  Asclepius::velocity_t velocity;
  hdGetFloatv(HD_CURRENT_VELOCITY, (HDfloat *)&velocity.data[0]);
  hdGetFloatv(HD_CURRENT_ANGULAR_VELOCITY, (HDfloat *)&velocity.data[3]);
  velocity.timestamp = std::chrono::steady_clock::now();
  buffer->velocity.add(velocity);

  // Write Force
  Asclepius::force_t force_buff;
  if (buffer->force.get(force_buff)) {
    float force[6];
    for (size_t i = 0; i < 6; i++)
      force[i] = (float)force_buff.data[i];
    hdSetFloatv(HD_CURRENT_FORCE, force);
    hdSetFloatv(HD_CURRENT_TORQUE, &force[3]);
  }
  hdEndFrame(buffer->device_id);

  loop_measurement.end = std::chrono::steady_clock::now();
  buffer->datalog->push(loop_measurement);
  return HD_CALLBACK_CONTINUE;
}

Asclepius::FrankaInterface::FrankaInterface(
    const std::string &hostname,
    const std::shared_ptr<Datalog<ThreadLoopMeasure, DATALOG_ELEMENTS_SIZE>>
        datalog)
    : m_datalog{datalog} {
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

Asclepius::FrankaInterface::~FrankaInterface() { stop(); }

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
    Asclepius::ThreadLoopMeasure loop_measurement{
        .begin{std::chrono::steady_clock::now()}, .end{}};
    if (Asclepius::g_shutdown_coordinator.is_shutdown()) {
      loop_measurement.end = std::chrono::steady_clock::now();
      m_datalog->push(loop_measurement);
      return franka::MotionFinished(franka::CartesianVelocities{0.0});
    }

    // Read Force 
    m_force.add({.data{state.O_F_ext_hat_K},
                 .timestamp{std::chrono::steady_clock::now()}});

    // Write Velocity 
    static std::array<double, 6> velocity = {0.0};
    velocity_t velocity_new;
    if (m_velocity.get(velocity_new)) {
      velocity = velocity_new.data;
    }
    loop_measurement.end = std::chrono::steady_clock::now();
    m_datalog->push(loop_measurement);
    return velocity;
  };
}

Asclepius::HapticInterface::HapticInterface(
    const std::string &devicename,
    const std::shared_ptr<Datalog<ThreadLoopMeasure, DATALOG_ELEMENTS_SIZE>>
        datalog) {
  m_buffer.datalog = datalog;
  HDErrorInfo error;
  m_buffer.device_id = hdInitDevice(devicename.c_str());
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Failed to initialize device.");
    throw AsclepiusException("Failed to initialize haptic device with name: " +
                             devicename);
  }
  hdEnable(HD_FORCE_OUTPUT);
}

Asclepius::HapticInterface::~HapticInterface() { stop(); }

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

Asclepius::Pipeline::Pipeline(
    const std::string &hostname, const std::string &devicename,
    const std::array<
        std::shared_ptr<Datalog<ThreadLoopMeasure, DATALOG_ELEMENTS_SIZE>>, 3>
        &datalogs)
    : m_franka_interface(hostname, datalogs.at(1)), m_haptic_interface(devicename, datalogs.at(2)),
      m_rotation_mapping(Eigen::Quaterniond::Identity()), m_datalog(datalogs.at(0)) {}

Asclepius::Pipeline::~Pipeline() { stop(); }

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
    init_impedance_filter(&filter, 1.0, 1.0, 1e-3);
  }
  FirstOrderHold<Asclepius::velocity_t> master_velocity_foh{1e-3};
  force_t slave_force_zoh;

  KilohertzLoop loop_timer;
  while (!g_shutdown_coordinator.is_shutdown()) {
    Asclepius::ThreadLoopMeasure loop_measurement{
        .begin{std::chrono::steady_clock::now()}, .end{}};
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

    m_franka_interface.m_velocity.add(slave_velocity);
    m_haptic_interface.m_buffer.force.add(master_force);

    loop_measurement.end = std::chrono::steady_clock::now();
    m_datalog->push(loop_measurement);
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
}*/
