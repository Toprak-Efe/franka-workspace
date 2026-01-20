project(cartesian
    VERSION 0.0.1
    DESCRIPTION "Cartesian control example for Franka Emika Panda robots."
    LANGUAGES CXX
)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES
    source/main.cpp
    source/examples_common.cpp
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

