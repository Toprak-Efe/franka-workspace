project(bilateralcontrol
    LANGUAGES CXX
)

set(HEADERS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bilateralcontrol/franka.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bilateralcontrol/haptic.hpp
    #${CMAKE_CURRENT_SOURCE_DIR}/include/bilateralcontrol/pipeline.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/bilateralcontrol/shutdown.hpp
)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/source/haptic.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/source/shutdown.cpp
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
    realtimeloop
    snippets
)

