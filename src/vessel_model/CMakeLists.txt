
add_library(vessel_model
    vessel_files.cpp
    model_2d.cpp
    model_3d.cpp
)
target_include_directories(vessel_model PUBLIC ${PROJECT_SOURCE_DIR}/src)

if(BUILD_TESTING)
    add_executable(test_vessel_model tests/test_vessel_model.cpp)
    target_link_libraries(test_vessel_model PRIVATE vessel_model testing_check)
    
    target_compile_definitions(test_vessel_model PRIVATE PROJECT_ROOT="${PROJECT_SOURCE_DIR}")
    add_test(NAME test_vessel_model COMMAND test_vessel_model)
endif()
