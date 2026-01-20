#pragma once

#include <atomic>
#include <array>
#include <unistd.h>
#include "cache.hpp"

namespace Asclepius {

template <class T>
class TripleBuffer {
public:
    TripleBuffer() :
        _producer_state({.index = 0}), _consumer_state({.index = 2})
    {
        _shared_state.index.store(1, std::memory_order_relaxed);
        _shared_state.new_data_available.store(false, std::memory_order_relaxed);
    } 

    void add(const T& data) {
        size_t write_idx = _producer_state.index;
        _buffers[write_idx] = data;
        _producer_state.index = _shared_state.index.exchange(write_idx, std::memory_order_acq_rel);
        _shared_state.new_data_available.store(true, std::memory_order_release);
    }

    bool get(T& data) {
        bool is_new = _shared_state.new_data_available.exchange(false, std::memory_order_acq_rel); 
        if (is_new) {
            size_t reader_idx = _consumer_state.index;
            _consumer_state.index = _shared_state.index.exchange(reader_idx, std::memory_order_acq_rel);
        }
        data = _buffers[_consumer_state.index];
        return is_new;
    }

private:
    std::array<T, 3> _buffers;

    struct alignas(CACHE_LINE) {
        std::atomic<size_t> index;
        std::atomic<bool> new_data_available; 
    } _shared_state; 

    struct alignas(CACHE_LINE) {
        size_t index;
    } _producer_state;
    
    struct alignas(CACHE_LINE) {
        size_t index;
    } _consumer_state;

}; // class TripleBuffer

} // namespace asclepius
