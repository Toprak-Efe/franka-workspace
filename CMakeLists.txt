cmake_minimum_required(VERSION 3.5)

set(CMAKE_C_COMPILER "clang-21")
set(CMAKE_CXX_COMPILER "clang++-21")
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(franka-workspace
    VERSION 0.0.1
    DESCRIPTION "Executables and libraries for a thesis project."
    LANGUAGES CXX
)

include(FetchContent)
FetchContent_Declare(
    json
    URL            https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
)
FetchContent_Declare(
    fmt
    GIT_REPOSITORY https://github.com/fmtlib/fmt
    GIT_TAG        e69e5f977d458f2650bb346dadf2ad30c5320281 # 10.2.1
) 
FetchContent_MakeAvailable(fmt json)


add_subdirectory(projects)
