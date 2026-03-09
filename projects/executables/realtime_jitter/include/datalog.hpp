#pragma once

#include <atomic>
#include <chrono>
#include <concepts>
#include <iterator>
#include <memory>
#include <stdexcept>

namespace Asclepius {

constexpr std::size_t DATALOG_ELEMENTS_SIZE = 65536;

template <typename T, std::size_t SIZE>
  requires std::default_initializable<T>
class Datalog {
public:
  struct Iterator {
    using iterator_category = std::forward_iterator_tag;
    using value_type = T;
    using element_type = value_type;
    using pointer = value_type *;
    using reference = value_type &;
    using difference_type = std::ptrdiff_t;

    Iterator() : _ptr{nullptr}, _idx(0) {};
    Iterator(Datalog *ptr, std::size_t idx) : _ptr{ptr}, _idx{idx} {};
    Iterator &operator++() {
      _idx++;
      return *this;
    }
    Iterator operator++(int) {
      Iterator tmp = *this;
      ++(*this);
      return tmp;
    }
    bool operator==(const Iterator &b) const {
      return _ptr == b._ptr && _idx == b._idx;
    }
    bool operator!=(const Iterator &b) const { return !(*this == b); }
    T *operator->() const { return &(**this); }
    T &operator*() const {
      if (_idx >= SIZE) {
        throw(std::out_of_range(
            "Dereferencing an element out of bounds is not allowed."));
      }
      return _ptr->m_data.buffer[_idx];
    }

  private:
    std::size_t _idx;
    Datalog *_ptr;
  };

  Datalog() {
    m_data.buffer = std::make_unique<T[]>(SIZE);
    m_data.end = 0;
  }

  auto begin() { return Iterator(this, 0); }
  auto end() { return Iterator(this, m_data.end); }

  void push(const T &data) {
    std::size_t pen = m_data.end.load(std::memory_order_relaxed);
    do {
      if (pen >= SIZE)
        return;
    } while (!m_data.end.compare_exchange_weak(
        pen, pen + 1, std::memory_order_relaxed, std::memory_order_relaxed));
    m_data.buffer[pen] = data;
  }

private:
  static_assert(std::forward_iterator<Iterator>);
  struct {
    std::unique_ptr<T[]> buffer;
    std::atomic_size_t end;
  } m_data;
};

struct ThreadLoopMeasure {
  std::chrono::time_point<std::chrono::steady_clock> begin, end;
};

using Timelog = Datalog<ThreadLoopMeasure, DATALOG_ELEMENTS_SIZE>;

}; // namespace Asclepius
