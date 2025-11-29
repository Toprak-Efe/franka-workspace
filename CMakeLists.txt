cmake_minimum_required(VERSION 3.5)
project(franka-workspace
    VERSION 0.0.1
    DESCRIPTION "Executables and libraries for a thesis project."
    LANGUAGES CXX
)

include(FetchContent)
FetchContent_Declare(cpptrace
  GIT_REPOSITORY https://github.com/jeremy-rifkin/cpptrace.git
  GIT_TAG        v1.0.4 # <HASH or TAG>
)
FetchContent_Declare(ftxui
  GIT_REPOSITORY https://github.com/ArthurSonzogni/ftxui
  GIT_TAG v6.1.9
)
FetchContent_MakeAvailable(cpptrace ftxui)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(projects)
