#include <mutex>
#include <shutdown.hpp>

Asclepius::ShutdownCoordinator::ShutdownCoordinator() : m_running(true) {}

void Asclepius::ShutdownCoordinator::shutdown() {
  std::lock_guard<std::mutex> lock(m_mutex);
  if (!m_running.exchange(true)) {
    return;
  }
  m_cv.notify_all();
}

bool Asclepius::ShutdownCoordinator::is_shutdown() {
  return m_running.load() == false;
}

void Asclepius::ShutdownCoordinator::await_shutdown() {
  std::unique_lock<std::mutex> lock(m_mutex);
  m_cv.wait(lock, [this] { return m_running.load(); });
}
