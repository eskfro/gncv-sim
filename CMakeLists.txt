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
    src/vessel.cpp
)
target_include_directories(gncv-sim-core PUBLIC src ${ARMADILLO_INCLUDE_DIRS})
target_link_libraries(gncv-sim-core PUBLIC ${ARMADILLO_LIBRARIES})

add_executable(sim2d
    apps/sim2d/main.cpp
    apps/sim2d/sim2d.cpp
)
target_link_libraries(sim2d PRIVATE gncv-sim-core)

if(BUILD_TESTING)
    add_executable(test_common
        tests/test_common.cpp
    )
    target_link_libraries(test_common PRIVATE gncv-sim-core)
    add_test(NAME test_common COMMAND test_common)
endif()
