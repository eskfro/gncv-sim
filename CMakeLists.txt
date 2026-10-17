cmake_minimum_required(VERSION 3.16)
project(gncv-sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Armadillo REQUIRED)
include(CTest)

add_subdirectory(src)

add_executable(simulator_v1
    apps/simulator_v1/main.cpp
)
target_link_libraries(simulator_v1 PRIVATE gncv-sim-core simdata)

add_subdirectory(apps/playback_2d)
add_subdirectory(apps/playback_3d)


