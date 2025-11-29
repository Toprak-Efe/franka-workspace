project(visualizer
    VERSION 0.0.1
    DESCRIPTION "A TUI visualizer for Franka robot joint states."
    LANGUAGES CXX
)

find_package(Franka REQUIRED)

set(SOURCES
    source/main.cpp
    source/menu.cpp
    source/app.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    Franka::Franka
    cpptrace::cpptrace
    ftxui::component
    ftxui::dom
    ftxui::screen
)

