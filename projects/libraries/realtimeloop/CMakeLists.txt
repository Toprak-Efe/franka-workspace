project(realtimeloop
    LANGUAGES CXX
)

set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/realtimeloop/monoloop.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/realtimeloop/schedtimer.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/realtimeloop/telemetry/asynclogger.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/realtimeloop/telemetry/loopinfo.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/realtimeloop/telemetry/scopedtimer.hpp
)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/source/exception.cpp
)

add_library(${PROJECT_NAME} STATIC)
target_include_directories(${PROJECT_NAME} PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_sources(${PROJECT_NAME}
    PUBLIC 
    ${HEADERS}
    PRIVATE
    ${SOURCES}
)
target_link_libraries(${PROJECT_NAME} PUBLIC 
    pthread
    atomic_queue
)

