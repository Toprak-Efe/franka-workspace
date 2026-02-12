project(state_reader
    VERSION 0.0.1
    DESCRIPTION "Read robot state to retrieve and print the control mode."
    LANGUAGES CXX
)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES
    source/main.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    pthread
    Franka::Franka
    Eigen3::Eigen
)

