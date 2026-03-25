project(franka_test)

set(SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/source/main.cpp
)
set(HEADERS
)

add_executable(${PROJECT_NAME})
target_sources(${PROJECT_NAME}
    PUBLIC
    ${HEADERS}
    PRIVATE
    ${SOURCES}
)
target_link_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    snippets
    nlohmann_json::nlohmann_json
    pthread
    atomic_queue
    Franka::Franka
    realtimeloop
    bilateralcontrol
    ${HDAPI_LIBRARIES}
)
