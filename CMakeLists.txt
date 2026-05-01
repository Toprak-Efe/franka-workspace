cmake_minimum_required(VERSION 4.0.0)

set(CMAKE_C_COMPILER "clang-21")
set(CMAKE_CXX_COMPILER "clang++-21")
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(franka-workspace
    VERSION 0.0.1
    DESCRIPTION "Libraries and executables for a thesis project."
    LANGUAGES CXX
)

find_package(Eigen3 REQUIRED)
find_package(Franka REQUIRED)

set(HD_LIBRARIES
    /usr/lib/libHD.so
    /usr/lib/libHDU.a
)

include(FetchContent)
set(FETCHCONTENT_UPDATES_DISCONNECTED ON)
FetchContent_Declare(
    matplotplusplus
    GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
    GIT_TAG v1.2.2
)
FetchContent_Declare(
    atomic_queue
    GIT_REPOSITORY https://github.com/max0x7ba/atomic_queue.git
    GIT_TAG v1.7.1
)
FetchContent_Declare(
    tlib
    GIT_REPOSITORY https://git.ninehundredandsixtyseventhhall.com/toprak/tlib.git
    GIT_TAG main
)
FetchContent_Declare(
  toml11
  GIT_REPOSITORY https://github.com/ToruNiina/toml11.git
  GIT_TAG        v4.4.0
)
FetchContent_MakeAvailable(matplotplusplus atomic_queue toml11 tlib)

add_subdirectory(projects)

