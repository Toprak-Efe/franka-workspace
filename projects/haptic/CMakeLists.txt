project(haptic
    VERSION 0.0.1
    DESCRIPTION "Force experiments for GeoMagic."
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(HDAPI_LIBRARIES
    /usr/lib/libHD.so
    /usr/lib/libHDU.a
)

set(SOURCES
    source/main.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${HDAPI_LIBRARIES}
)

