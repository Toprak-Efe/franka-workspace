project(controller
    VERSION 0.0.1
    DESCRIPTION "Main controller for the franka robots."
    LANGUAGES CXX
)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

set(HDAPI_LIBRARIES
    /usr/lib/libHD.so
    /usr/lib/libHDU.a
)

set(SOURCES
    source/exception.cpp
    source/shutdown.cpp
    source/pipeline.cpp
    source/filter.cpp
    source/config.cpp
    source/main.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    pthread
    Franka::Franka
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
    realtime_loop
    ${HDAPI_LIBRARIES}
)

