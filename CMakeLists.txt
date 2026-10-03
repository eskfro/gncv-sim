cmake_minimum_required(VERSION 3.16)
project(gncv-sim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Armadillo REQUIRED)
include(CTest)

add_library(gncv-sim-core
    src/common.cpp
    src/actuator.cpp
    src/sensor.cpp
    src/sensor_io.cpp
    src/dynamics.cpp
    src/guidance.cpp
    src/controller.cpp
    src/estimation.cpp
    src/thrust_allocator.cpp
    src/vessel.cpp
)
target_include_directories(gncv-sim-core PUBLIC src ${ARMADILLO_INCLUDE_DIRS})
target_link_libraries(gncv-sim-core PUBLIC ${ARMADILLO_LIBRARIES})

# Libraries shared by the apps (simdata, vessel_model, playback), see lib/
add_subdirectory(lib)

add_executable(simulator_v1
    apps/simulator_v1/main.cpp
)
target_link_libraries(simulator_v1 PRIVATE gncv-sim-core simdata)

# Apps with their own CMakeLists.txt
add_subdirectory(apps/playback_2d)
add_subdirectory(apps/playback_3d)

if(BUILD_TESTING)
    add_executable(test_common
        tests/test_common.cpp
    )
    target_link_libraries(test_common PRIVATE gncv-sim-core)
    add_test(NAME test_common COMMAND test_common)
endif()
