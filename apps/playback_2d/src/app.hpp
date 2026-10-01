#pragma once
// ============================================================================
// Window, event loop and layout (SDL2 + Dear ImGui)
// ============================================================================
#include <filesystem>

namespace playback2d {

struct Options {
    std::filesystem::path input;  // csv file or data directory, empty = auto
    double speed{1.0};
    bool follow{false};
    bool loop{false};
    float ui_scale{1.0f};
    // Render one frame at screenshot_time and save it as BMP, then exit
    std::filesystem::path screenshot;
    double screenshot_time{0.0};
};

// Returns the process exit code
int Run(const Options& options);

}  // namespace playback2d
