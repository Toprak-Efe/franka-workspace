#pragma once

#include <atomic>
#include <condition_variable>
#include <exception>
#include <mutex>

namespace Asclepius {

class ShutdownCoordinator {
public:
  ShutdownCoordinator();

  void shutdown() {
    if (!m_running.exchange(true)) {
      return;
    }

    std::lock_guard<std::mutex> lock(m_mutex);
    m_cv.notify_all();
  }

  void shutdown_exception(std::exception_ptr exception) {
    if (m_exception_block.exset.exchange(true)) {
      return;
    }

    m_exception_block.exception = exception;
    shutdown();
  }

  bool is_shutdown() { return !m_running.load(); }

  void await_shutdown() {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [this] { return m_running.load(); });

    if (m_exception_block.exset) {
      std::rethrow_exception(m_exception_block.exception);
    }
  }

private:
  struct {
    std::atomic_bool exset{false};
    std::exception_ptr exception{nullptr};
  } m_exception_block;
  std::mutex m_mutex;
  std::atomic_bool m_running;
  std::condition_variable m_cv;
};

inline ShutdownCoordinator g_shutdown_coordinator;

}; // namespace Asclepius
