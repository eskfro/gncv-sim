# vessel_model: the vessels/<name>/ folder and the models in it. No GUI dependencies.
#
#   #include "vessel_model/vessel_files.hpp"  vessels/ lookup
#   #include "vessel_model/model_2d.hpp"      top view from model_2d.svg
#   #include "vessel_model/model_3d.hpp"      mesh from model_3d.obj (+ .mtl)
#   #include "vessel_model/parts.hpp"         parts that move with a csv column
add_library(vessel_model
    vessel_files.cpp
    model_2d.cpp
    model_3d.cpp
)
target_include_directories(vessel_model PUBLIC ${PROJECT_SOURCE_DIR}/lib)

if(BUILD_TESTING)
    add_executable(test_vessel_model tests/test_vessel_model.cpp)
    target_link_libraries(test_vessel_model PRIVATE vessel_model testing_check)
    # The tests also check that every model in vessels/ loads
    target_compile_definitions(test_vessel_model PRIVATE PROJECT_ROOT="${PROJECT_SOURCE_DIR}")
    add_test(NAME test_vessel_model COMMAND test_vessel_model)
endif()
