#include <bilateralcontrol/shutdown.hpp>

void Asclepius::ShutdownCoordinator::shutdown() {
  if (!m_running.exchange(false)) {
    return;
  }

  std::lock_guard<std::mutex> lock(m_mutex);
  m_cv.notify_all();
}

void Asclepius::ShutdownCoordinator::shutdown_with_exception(const std::exception_ptr &exception) {
  if (!m_running.exchange(false)) {
    return;
  }
  m_exception = std::move(exception);
  std::lock_guard<std::mutex> lock(m_mutex);
  m_cv.notify_all();
}

bool Asclepius::ShutdownCoordinator::is_shutdown() { return !m_running.load(); }

void Asclepius::ShutdownCoordinator::await_shutdown() {
  std::unique_lock<std::mutex> lock(m_mutex);
  m_cv.wait(lock, [this] { return m_running.load(); });

  if (m_exception) {
    std::rethrow_exception(m_exception);
  }
}

Asclepius::ShutdownCoordinator::ShutdownCoordinator() {}

Asclepius::ShutdownCoordinator &Asclepius::ShutdownCoordinator::instance() {
  static Asclepius::ShutdownCoordinator g_coordinator{};
  return g_coordinator;
}

std::exception_ptr Asclepius::ShutdownCoordinator::get_shutdown_exception() { return m_exception; }
