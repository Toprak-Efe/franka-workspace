project(realtime_jitter
    VERSION 0.0.1
    DESCRIPTION "Baseline 1MS loop measurements."
    LANGUAGES CXX
)

set(SOURCES
    source/main.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    pthread
    Matplot++::matplot
)

