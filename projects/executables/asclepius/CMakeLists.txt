project(asclepius
    VERSION 0.0.1
    DESCRIPTION "Main controller for the franka robots."
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

set(HDAPI_LIBRARIES
    /usr/lib/libHD.so
    /usr/lib/libHDU.a
)

set(SOURCES
    source/main.cpp
    source/config.cpp
    source/shutdown.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    pthread
    atomic_queue
    Franka::Franka
    nlohmann_json::nlohmann_json
    Matplot++::matplot
    Eigen3::Eigen
    realtime_loop
    ${HDAPI_LIBRARIES}
)

