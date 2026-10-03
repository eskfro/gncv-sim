# playback: shared by the playback apps.
#
#   playback     PlaybackClock and Session (runs, loaded recording, vessel).
#                No GUI dependencies, always built and tested.
#   playback_ui  ImGui widgets (run list, telemetry, playback bar). Only
#                built when SDL2 is installed.
add_library(playback
    playback_clock.cpp
    session.cpp
)
target_include_directories(playback PUBLIC ${PROJECT_SOURCE_DIR}/lib)
target_link_libraries(playback PUBLIC simdata vessel_model)

if(BUILD_TESTING)
    add_executable(test_playback tests/test_playback.cpp)
    target_link_libraries(test_playback PRIVATE playback testing_check)
    add_test(NAME test_playback COMMAND test_playback)
endif()

find_package(SDL2 CONFIG QUIET)
if(SDL2_FOUND)
    include(${PROJECT_SOURCE_DIR}/cmake/imgui.cmake)
    add_library(playback_ui widgets.cpp)
    target_link_libraries(playback_ui PUBLIC playback imgui)
endif()
