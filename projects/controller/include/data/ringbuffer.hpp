#pragma once

#include <atomic>
#include <array>
#include <bit>
#include "cache.hpp"

namespace Asclepius {

template<class T, std::size_t L>
class RingBuffer {
    static_assert(L > 1, "Ring buffer size L must be large than 1.");
    static_assert(std::__has_single_bit(L), "Size L must be a power of 2."); 
public:
    RingBuffer() : _writer(0), _reader(0) {}
    bool get(T& data) {
        const auto curr_reader = _reader.load(std::memory_order_relaxed);
        const auto curr_writer = _writer.load(std::memory_order_acquire);
        if (curr_reader == curr_writer) {
            return false;
        }
        data = std::move(_buffer[curr_reader]);
        const auto next_reader = (curr_reader + 1) & (L - 1);
        _reader.store(next_reader, std::memory_order_release);
        return true;
    } 
    bool add(const T &data) {
        const auto curr_writer = _writer.load(std::memory_order_relaxed);
        const auto next_writer = (curr_writer + 1) & (L - 1);
        if (next_writer == _reader.load(std::memory_order_acquire)) {
            return false;
        }
        _buffer[curr_writer] = data;
        _writer.store(next_writer, std::memory_order_release);
        return true;
    }
private:
    std::array<T, L> _buffer;
    alignas(CACHE_LINE) std::atomic<std::size_t> _writer;
    alignas(CACHE_LINE) std::atomic<std::size_t> _reader;
}; // RingBuffer 

}; // namespace Asclepius

