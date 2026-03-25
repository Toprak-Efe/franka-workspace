#include <HD/hdDefines.h>
#include <HD/hdDevice.h>
#include <HD/hdScheduler.h>
#include <Eigen/src/Core/Matrix.h>
#include <bilateralcontrol/haptic.hpp>
#include <mutex>

HDCallbackCode HDCALLBACK asclepius_haptic_callback(void *buffer) {
  using namespace Asclepius;
  HapticBuffer *buff = reinterpret_cast<HapticBuffer *>(buffer);

  hdMakeCurrentDevice(buff->device_handle);
  hdBeginFrame(buff->device_handle);
  /* Read Velocity */
  TwistDisplacement td{};
  Eigen::Matrix<float, 3, 1> v, w;
  hdGetFloatv(HD_CURRENT_VELOCITY, (HDfloat *)Eigen::Map<std::array<float, 3>>(v));
  hdGetFloatv(HD_CURRENT_ANGULAR_VELOCITY, (HDfloat *)w.data());
  std::array<float, 3> p, o;
  hdGetFloatv(HD_CURRENT_POSITION, (HDfloat *)p.data());
  hdGetFloatv(HD_CURRENT_GIMBAL_ANGLES, (HDfloat *)o.data());
  Eigen::Matrix<float, 6, 1> vw;

  t.stamp() = std::chrono::steady_clock::now();

  buff->buffer.tdbuff buff->force_velocity_buffer.velocity.add(vw);

  /* Write Force */
  Force ft{};
  if (buff->force_velocity_buffer.force.get(ft)) {
    buff->zoh.push(ft);
  } else {
    ft = buff->zoh.sample();
  }
  float force[6];
  for (size_t i = 0; i < 6; i++)
    force[i] = static_cast<float>(ft.data[i]);
  hdSetFloatv(HD_CURRENT_FORCE, force);
  hdSetFloatv(HD_CURRENT_TORQUE, &force[3]);
  hdEndFrame(buff->device_handle);

  if (HD_DEVICE_ERROR(buff->error = hdGetError())) {
    return HD_CALLBACK_DONE;
  }

  if (buff->p_stop_token->stop_requested()) {
    return HD_CALLBACK_DONE;
  } else {
    return HD_CALLBACK_CONTINUE;
  }
}

void Asclepius::HapticSchedulerCounter::incremenent() {
  std::lock_guard<std::mutex> guard{m_lock};
  if (m_scheduled++ == 0) {
    hdStartScheduler();
  }
}

void Asclepius::HapticSchedulerCounter::decrement() {
  std::lock_guard<std::mutex> guard{m_lock};
  if (--m_scheduled == 0) {
    hdStopScheduler();
  }
}
