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

include(FetchContent)
FetchContent_Declare(
    Matplot++
    GIT_REPOSITORY https://github.com/alandefreitas/matplotplusplus
    GIT_TAG v1.2.2
)
FetchContent_Declare(
    atomic_queue
    GIT_REPOSITORY https://github.com/max0x7ba/atomic_queue.git
    GIT_TAG v1.7.1
)
FetchContent_Declare(
    json
    URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
)
FetchContent_MakeAvailable(Matplot++ atomic_queue json)

add_subdirectory(projects)
