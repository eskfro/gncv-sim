# Libraries shared by the apps. Include them as "<library>/<header>.hpp".
#
#   simdata       simulation runs on disk: writing and reading      (no GUI)
#   vessel_model  vessels/<name>/ lookup, 2D (SVG) and 3D (OBJ) models (no GUI)
#   playback      playback clock and session (no GUI), plus
#   playback_ui   ImGui widgets shared by the playback apps (needs SDL2)
add_subdirectory(simdata)
add_subdirectory(vessel_model)
add_subdirectory(playback)

# testing/check.hpp, the CHECK() macro used by the unit tests
add_library(testing_check INTERFACE)
target_include_directories(testing_check INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
