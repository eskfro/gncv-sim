# playback_3d: replays simulation runs as a 3D animation of the vessel.
#
# Targets
#   playback_3d_model : math, orbit camera, vessel pose and scene building.
#                       No GUI or OpenGL dependencies, always built and tested.
#   playback_3d       : SDL2 + OpenGL 3.3 + Dear ImGui front end. Skipped with a
#                       warning when SDL2 or OpenGL is not installed
#                       (sudo apt install libsdl2-dev libgl-dev).
#   test_playback_3d  : unit tests for playback_3d_model
#
# Runs, csv loading and the playback clock come from lib/ (simdata, playback),
# the vessel mesh from vessels/<name>/model_3d.obj (vessel_model).
# Does not use gncv-sim-core: it only reads the files simulator_v1 writes.

add_library(playback_3d_model
    src/orbit_camera.cpp
    src/scene.cpp
    src/vessel_pose.cpp
)
target_include_directories(playback_3d_model PUBLIC src)
target_link_libraries(playback_3d_model PUBLIC simdata vessel_model)

if(BUILD_TESTING)
    add_executable(test_playback_3d tests/test_playback_3d.cpp)
    target_link_libraries(test_playback_3d PRIVATE playback_3d_model testing_check)
    add_test(NAME test_playback_3d COMMAND test_playback_3d)
endif()

if(NOT TARGET playback_ui OR NOT TARGET imgui_opengl3)
    message(WARNING "playback_3d: SDL2 or OpenGL not found, skipping the GUI. Install them with: sudo apt install libsdl2-dev libgl-dev")
    return()
endif()

add_executable(playback_3d
    src/main.cpp
    src/app.cpp
    src/app_state.cpp
    src/layers.cpp
    src/panels.cpp
    src/renderer.cpp
)
target_link_libraries(playback_3d PRIVATE playback_3d_model playback_ui imgui_opengl3)
# Next to the other apps (build/simulator_v1, build/playback_2d)
set_target_properties(playback_3d PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
