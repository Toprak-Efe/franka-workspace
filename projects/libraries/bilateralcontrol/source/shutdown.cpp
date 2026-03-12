#include <bilateralcontrol/shutdown.hpp>

void Asclepius::ShutdownCoordinator::shutdown() {
  if (!m_running.exchange(false)) {
    return;
  }

  std::lock_guard<std::mutex> lock(m_mutex);
  m_cv.notify_all();
}

void Asclepius::ShutdownCoordinator::shutdown_exception(std::exception_ptr &exception) {
  if (m_exception_block.exset.exchange(true)) {
    return;
  }

  m_exception_block.exception = std::move(exception);
  shutdown();
}

bool Asclepius::ShutdownCoordinator::is_shutdown() { return !m_running.load(); }

void Asclepius::ShutdownCoordinator::await_shutdown() {
  std::unique_lock<std::mutex> lock(m_mutex);
  m_cv.wait(lock, [this] { return m_running.load(); });

  if (m_exception_block.exset) {
    std::rethrow_exception(m_exception_block.exception);
  }
}

Asclepius::ShutdownCoordinator::ShutdownCoordinator() {}

Asclepius::ShutdownCoordinator &Asclepius::ShutdownCoordinator::get() {
  static Asclepius::ShutdownCoordinator g_coordinator{};
  return g_coordinator;
}

bool Asclepius::ShutdownCoordinator::is_shutdown_exception() {
    return m_exception_block.exset.load(std::memory_order_relaxed) && m_exception_block.exset;
}

std::optional<std::exception_ptr> Asclepius::ShutdownCoordinator::get_shutdown_exception() {
    if (m_exception_block.exset) {
        return m_exception_block.exception;
    }
    return {};
}
