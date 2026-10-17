add_subdirectory(core)
add_subdirectory(simdata)
add_subdirectory(vessel_model)
add_subdirectory(playback)

add_library(testing_check INTERFACE)
target_include_directories(testing_check INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
