cmake_minimum_required(VERSION 3.14)
project(drone_companion CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fetch Raylib via FetchContent
include(FetchContent)
FetchContent_Declare(
    raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/5.0.tar.gz
)
FetchContent_MakeAvailable(raylib)

add_executable(drone_viewer
    src/main.cpp
)

target_link_libraries(drone_viewer PRIVATE raylib)
target_include_directories(drone_viewer PRIVATE ../shared)

if(UNIX AND NOT APPLE)
    target_link_libraries(drone_viewer PRIVATE m pthread dl rt X11)
endif()
