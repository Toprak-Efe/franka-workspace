#pragma once

#include <atomic>
#include <atomic_queue/atomic_queue.h>
#include <semaphore>
#include <thread>
#include <type_traits>
#include <vector>

namespace Asclepius::Telemetry {

template <typename T, std::size_t L>
requires std::is_trivially_constructible_v<T>
class AsyncLogger {
public:
  AsyncLogger() : m_drop_count{0}, m_wake{0}, m_write_buffer{} { m_write_buffer.reserve(65'536); }
  ~AsyncLogger() { stop(); }

  void push(const T &data) {
    if (!m_write_queue.try_push(data)) {
      m_drop_count.fetch_add(1);
    } else {
      m_wake.release();
    }
  }

  void start() {
    if (m_thread.joinable())
      return;
    m_running.store(true, std::memory_order_relaxed);
    m_wake.try_acquire();
    m_thread = std::thread{[this]() {
      using namespace std::chrono_literals;
      while (m_running.load(std::memory_order_relaxed)) {
        drain_queue();
        m_wake.try_acquire_for(100ms);
      }
      drain_queue();
    }};
  }

  void stop() {
    m_running.store(false, std::memory_order_relaxed);
    m_wake.release(1);
    if (m_thread.joinable()) {
      m_thread.join();
    }
  }

  std::vector<T> &get_buffer() { return m_write_buffer; }

  uint32_t get_drop_count() { return m_drop_count; }

private:
  void drain_queue() {
    T data{};
    while (m_write_queue.try_pop(data)) {
      m_write_buffer.emplace_back(std::move(data));
    }
  }
  std::atomic_bool m_running{false};
  std::counting_semaphore<L> m_wake{0};
  std::atomic<uint32_t> m_drop_count{0};
  atomic_queue::AtomicQueue<T, L> m_write_queue;
  std::vector<T> m_write_buffer;
  std::thread m_thread;
}; // class AsyncLogger

struct EmptyLogger {
  template <typename... Args> EmptyLogger(Args &&...) {}
};

}; // namespace Asclepius::Telemetry
