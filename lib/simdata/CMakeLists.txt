# simdata: simulation runs on disk (simdata/<run>/metadata.json + simulation.csv)
# and reading them back. No GUI dependencies.
#
#   #include "simdata/run.hpp"        run folders, metadata, RunWriter, ListRuns
#   #include "simdata/recording.hpp"  csv loader with interpolation
#   #include "simdata/channels.hpp"   known column names
#   #include "simdata/paths.hpp"      project root, simdata directory
include(${PROJECT_SOURCE_DIR}/cmake/nlohmann_json.cmake)

add_library(simdata
    paths.cpp
    recording.cpp
    run.cpp
)
target_include_directories(simdata PUBLIC ${PROJECT_SOURCE_DIR}/lib)
target_link_libraries(simdata PUBLIC nlohmann_json::nlohmann_json)

if(BUILD_TESTING)
    add_executable(test_simdata tests/test_simdata.cpp)
    target_link_libraries(test_simdata PRIVATE simdata testing_check)
    add_test(NAME test_simdata COMMAND test_simdata)
endif()
