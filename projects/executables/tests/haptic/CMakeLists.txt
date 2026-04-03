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

add_executable(analysis)
target_sources(analysis
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/source/analysis.cpp
)
target_link_libraries(analysis PRIVATE
    tlib
    matplot
    Eigen3::Eigen
    toml11::toml11
    bilateralcontrol
    realtimeloop
)
target_include_directories(analysis PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

