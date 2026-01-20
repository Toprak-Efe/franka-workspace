project(controller
    VERSION 0.0.1
    DESCRIPTION "Main controller for the franka robots."
    LANGUAGES CXX
)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(HDAPI_LIBRARIES
    /usr/lib/libHD.so
)

set(SOURCES
    source/main.cpp
    source/config.cpp
    source/app.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    pthread
    Franka::Franka
    fmt::fmt
    cpptrace::cpptrace
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
    ${HDAPI_LIBRARIES}
)

