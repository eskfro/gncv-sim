# Dear ImGui with the SDL2 + SDL_Renderer backends, as the static library
# target `imgui`. Shared by every GUI app: include() this file and link `imgui`.
#
# Sources are downloaded at configure time. To build offline, point CMake at
# a local checkout: -DFETCHCONTENT_SOURCE_DIR_IMGUI=/path/to/imgui
include_guard(GLOBAL)

find_package(SDL2 REQUIRED CONFIG)

include(FetchContent)
FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.9
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(imgui)

add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdlrenderer2.cpp
)
# SYSTEM: don't report warnings from third party headers
target_include_directories(imgui SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(imgui PUBLIC SDL2::SDL2)
