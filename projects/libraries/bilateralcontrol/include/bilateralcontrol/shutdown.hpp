#pragma once

#include <atomic>
#include <condition_variable>
#include <exception>
#include <mutex>

namespace Asclepius {

class ShutdownCoordinator {
public:
  void await_shutdown();

  void shutdown();
  void shutdown_with_exception(const std::exception_ptr &exception);

  bool is_shutdown();
  std::exception_ptr get_shutdown_exception();

  static ShutdownCoordinator &instance();

private:
  ShutdownCoordinator();

  std::mutex m_mutex;
  std::exception_ptr m_exception{nullptr};
  std::atomic_bool m_running{true};
  std::condition_variable m_cv;
};

}; // namespace Asclepius

