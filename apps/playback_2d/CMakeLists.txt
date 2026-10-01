# playback_2d: replays simulator_v1 csv files as a top-down 2D animation.
#
# Targets
#   playback_2d_model : csv loading, interpolation, playback clock, camera.
#                       No GUI dependencies, so it is always built and tested.
#   playback_2d       : SDL2 + Dear ImGui front end. Skipped with a warning
#                       when SDL2 is not installed (sudo apt install libsdl2-dev).
#   test_playback_2d  : unit tests for playback_2d_model
#
# Does not use gncv-sim-core: it only reads the csv files simulator_v1 writes.

add_library(playback_2d_model
    src/recording.cpp
    src/playback_clock.cpp
    src/camera.cpp
    src/data_files.cpp
)
target_include_directories(playback_2d_model PUBLIC src)

if(BUILD_TESTING)
    add_executable(test_playback_2d tests/test_playback_2d.cpp)
    target_link_libraries(test_playback_2d PRIVATE playback_2d_model)
    add_test(NAME test_playback_2d COMMAND test_playback_2d)
endif()

find_package(SDL2 CONFIG QUIET)
if(NOT SDL2_FOUND)
    message(WARNING "playback_2d: SDL2 not found, skipping the GUI. Install it with: sudo apt install libsdl2-dev")
    return()
endif()

include(${PROJECT_SOURCE_DIR}/cmake/imgui.cmake)

add_executable(playback_2d
    src/main.cpp
    src/app.cpp
    src/app_state.cpp
    src/layers.cpp
    src/panels.cpp
)
target_link_libraries(playback_2d PRIVATE playback_2d_model imgui)
# Next to the other apps (build/simulator_v1, build/sim2d)
set_target_properties(playback_2d PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
