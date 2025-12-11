project(visualizer
    VERSION 0.0.1
    DESCRIPTION "A TUI visualizer for Franka robot joint states."
    LANGUAGES CXX
)

find_package(Franka REQUIRED)
find_package(Nebula REQUIRED)

set(SOURCES
    source/config.cpp
    source/entry.cpp
    source/menu.cpp
    source/app.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    Nebula
    SDL2::SDL2
    OpenGL::OpenGL
    Franka::Franka
    cpptrace::cpptrace
    nlohmann_json::nlohmann_json
)

