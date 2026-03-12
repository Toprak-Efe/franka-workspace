#pragma once

#include <atomic>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <optional>

namespace Asclepius {

class ShutdownCoordinator {
public:
  void shutdown();
  void shutdown_exception(std::exception_ptr &exception);
  void await_shutdown();
  bool is_shutdown();
  bool is_shutdown_exception();
  std::optional<std::exception_ptr> get_shutdown_exception();
  static ShutdownCoordinator &get();

private:
  ShutdownCoordinator();
  struct {
    std::atomic_bool exset{false};
    std::exception_ptr exception{nullptr};
  } m_exception_block;
  std::mutex m_mutex;
  std::atomic_bool m_running{true};
  std::condition_variable m_cv;
};

}; // namespace Asclepius
