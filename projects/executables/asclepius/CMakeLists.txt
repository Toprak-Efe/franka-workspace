project(asclepius
    VERSION 0.0.1
    DESCRIPTION "Main controller for the franka robots."
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SOURCES
    source/main.cpp
    source/config.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    Eigen3::Eigen
    toml11::toml11
    pthread
    atomic_queue
    Franka::Franka
    realtimeloop
    bilateralcontrol
    ${HD_LIBRARIES}
)

