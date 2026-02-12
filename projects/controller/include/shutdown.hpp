#include <atomic>
#include <condition_variable>
#include <mutex>

namespace Asclepius {

class ShutdownCoordinator {
public:
  ShutdownCoordinator();
  void shutdown();
  bool is_shutdown();
  void await_shutdown();

private:
  std::mutex m_mutex;
  std::atomic_bool m_running;
  std::condition_variable m_cv;
};

inline ShutdownCoordinator g_shutdown_coordinator;

}; // namespace Asclepius
