# Dear ImGui for the GUI apps, as static library targets:
#
#   imgui              core + SDL2 platform backend (windows, input)
#   imgui_sdlrenderer  + SDL_Renderer backend, for 2D apps (playback_2d)
#   imgui_opengl3      + OpenGL 3 backend, for 3D apps (playback_3d)
#
# include() this file and link imgui_sdlrenderer or imgui_opengl3. Libraries
# with widgets only (no backend) link imgui.
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
)
# SYSTEM: don't report warnings from third party headers
target_include_directories(imgui SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(imgui PUBLIC SDL2::SDL2)

add_library(imgui_sdlrenderer STATIC ${imgui_SOURCE_DIR}/backends/imgui_impl_sdlrenderer2.cpp)
target_link_libraries(imgui_sdlrenderer PUBLIC imgui)

# OpenGL is optional: without it only the 3D apps are skipped
find_package(OpenGL QUIET)
if(OpenGL_FOUND)
    add_library(imgui_opengl3 STATIC ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
    target_link_libraries(imgui_opengl3 PUBLIC imgui OpenGL::GL)
endif()
