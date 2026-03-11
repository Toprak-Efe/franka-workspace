#pragma once

#include <cstring>
#include <realtimeloop/control/cache.hpp>
#include <array>
#include <atomic>
#include <unistd.h>

namespace Asclepius::Control {

template <class T> class TripleBuffer {
private:
public:
  TripleBuffer()
      : m_procuder_state({.index = 0}), m_consumer_state({.index = 2}) {
    m_shared_state.index.store(1, std::memory_order_relaxed);
    m_shared_state.new_data_available.store(false, std::memory_order_relaxed);
  }

  void add(const T &data) {
    size_t write_idx = m_procuder_state.index;
    m_buffers[write_idx].data = data;
    m_procuder_state.index =
        m_shared_state.index.exchange(write_idx, std::memory_order_acq_rel);
    m_shared_state.new_data_available.store(true, std::memory_order_release);
  }

  bool get(T &data) {
    bool is_new = m_shared_state.new_data_available.exchange(
        false, std::memory_order_acq_rel);
    if (is_new) {
      size_t reader_idx = m_consumer_state.index;
      m_consumer_state.index =
          m_shared_state.index.exchange(reader_idx, std::memory_order_acq_rel);
    }
    data = m_buffers[m_consumer_state.index].data;
    return is_new;
  }

  void reset() {
    memset(m_buffers.data(), 0, 3*sizeof(PaddedSlot));
  }

private:
  struct alignas(CACHE_LINE) PaddedSlot {
    T data;
  };
  std::array<PaddedSlot, 3> m_buffers;

  struct alignas(CACHE_LINE) {
    std::atomic<size_t> index;
    std::atomic<bool> new_data_available;
  } m_shared_state;

  struct alignas(CACHE_LINE) {
    size_t index;
  } m_procuder_state;

  struct alignas(CACHE_LINE) {
    size_t index;
  } m_consumer_state;

}; // class TripleBuffer

}; // namespace Asclepius::Control
