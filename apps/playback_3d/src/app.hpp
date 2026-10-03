#pragma once
// ============================================================================
// Window, event loop and layout (SDL2 + OpenGL 3.3 + Dear ImGui)
// ============================================================================
#include <filesystem>
#include <string>

namespace playback3d {

struct Options {
    std::filesystem::path input;  // run folder, csv or simdata directory, empty = auto
    std::string vessel;           // draw this vessel for every run, empty = from metadata
    double speed{1.0};
    bool follow{true};
    bool chase{false};
    bool loop{false};
    double attitude_scale{1.0};   // exaggerate roll and pitch
    float ui_scale{1.0f};
    // Render one frame at screenshot_time and save it as BMP, then exit
    std::filesystem::path screenshot;
    double screenshot_time{0.0};
    bool screenshot_fit{false};   // fit the whole track instead of the default view
};

// Returns the process exit code
int Run(const Options& options);

}  // namespace playback3d
