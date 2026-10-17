include(${PROJECT_SOURCE_DIR}/cmake/nlohmann_json.cmake)

add_library(simdata
    paths.cpp
    recording.cpp
    run.cpp
)
target_include_directories(simdata PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(simdata PUBLIC nlohmann_json::nlohmann_json)

if(BUILD_TESTING)
    add_executable(test_simdata tests/test_simdata.cpp)
    target_link_libraries(test_simdata PRIVATE simdata testing_check)
    add_test(NAME test_simdata COMMAND test_simdata)
endif()
