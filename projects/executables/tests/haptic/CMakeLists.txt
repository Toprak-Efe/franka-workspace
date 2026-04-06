project(haptic_test)

add_executable(haptic_test)
target_sources(haptic_test
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/source/main.cpp
)
target_link_libraries(haptic_test PRIVATE
    tlib
    Eigen3::Eigen
    toml11::toml11
    bilateralcontrol
    realtimeloop
    ${HDAPI_LIBRARIES}
)
target_include_directories(haptic_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(graph_latest)
target_sources(graph_latest
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/source/graph_latest.cpp
)
target_link_libraries(graph_latest PRIVATE
    tlib
    matplot
    Eigen3::Eigen
    toml11::toml11
    bilateralcontrol
    realtimeloop
)
target_include_directories(graph_latest PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(lowpass_graph)
target_sources(lowpass_graph
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/source/lowpass_graph.cpp
)
target_link_libraries(lowpass_graph PRIVATE
    tlib
    matplot
    Eigen3::Eigen
    toml11::toml11
    bilateralcontrol
    realtimeloop
)
target_include_directories(lowpass_graph PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

