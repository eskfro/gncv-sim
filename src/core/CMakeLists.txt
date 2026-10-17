add_library(gncv-sim-core
    common.cpp
    actuator.cpp
    sensor.cpp
    sensor_io.cpp
    dynamics.cpp
    guidance.cpp
    controller.cpp
    estimation.cpp
    thrust_allocator.cpp
    vessel.cpp
)

target_include_directories(gncv-sim-core PUBLIC ${PROJECT_SOURCE_DIR}/src ${ARMADILLO_INCLUDE_DIRS})
target_link_libraries(gncv-sim-core PUBLIC ${ARMADILLO_LIBRARIES})

if(BUILD_TESTING)
    add_executable(test_common
        tests/test_common.cpp
    )
    target_link_libraries(test_common PRIVATE gncv-sim-core)
    add_test(NAME test_common COMMAND test_common)
endif()