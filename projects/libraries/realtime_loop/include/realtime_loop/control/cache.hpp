#pragma once

#include <new>

#ifdef __cpp_lib_hardware_interference_size 
    constexpr std::size_t CACHE_LINE = std::hardware_destructive_interference_size;
#else
    constexpr std::size_t CACHE_LINE = 64; 
#endif
