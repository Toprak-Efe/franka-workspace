#pragma once

#include <atomic>
#include <atomic_queue/atomic_queue.h>
#include <mutex>
#include <thread>
#include <type_traits>
#include <vector>

namespace Asclepius::Telemetry {

template <typename T, std::size_t L>
requires std::is_trivially_constructible_v<T> && std::is_copy_constructible_v<T>
class AsyncLogger {
public:
  AsyncLogger() : m_drop_count{0}, m_write_buffer{} { m_write_buffer.reserve(65'536); }
  ~AsyncLogger() { stop(); }

  void push(const T &data) {
    if (!m_write_queue.try_push(data)) {
      m_drop_count.fetch_add(1);
    }
  }

  void start() {
    std::lock_guard<std::mutex> guard{m_mutex};
    if (m_running) {
      return;
    } else {
      m_running = true;
    };

    m_thread = std::jthread{[&](std::stop_token stoken) {
      using namespace std::chrono_literals;
      while (!stoken.stop_requested()) {
        drain_queue();
        std::this_thread::sleep_for(100ms);
      }
      drain_queue();
      m_running = false;
    }};
  }

  void stop() {
    std::jthread local_thread;
    {
      std::lock_guard<std::mutex> guard{m_mutex};
      std::swap(local_thread, m_thread);
    }
    m_thread.request_stop();
    if (m_thread.joinable()) {
      m_thread.join();
    }
  }

  /**
   * @note NOT THREAD SAFE.
   */
  std::vector<T> &get_buffer() { return m_write_buffer; }

  uint32_t get_drop_count() { return m_drop_count.load(std::memory_order_relaxed); }

private:
  void drain_queue() {
    T data{};
    while (m_write_queue.try_pop(data)) {
      m_write_buffer.emplace_back(std::move(data));
    }
  }

  std::mutex m_mutex;
  bool m_running{false};
  std::jthread m_thread;

  std::vector<T> m_write_buffer;
  atomic_queue::AtomicQueue<T, L> m_write_queue;
  std::atomic_uint32_t m_drop_count{0};
}; // class AsyncLogger

}; // namespace Asclepius::Telemetry
