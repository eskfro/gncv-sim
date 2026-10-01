#pragma once
// ============================================================================
// Everything the UI panels read and change. Owned by App.
// ============================================================================
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "camera.hpp"
#include "channels.hpp"
#include "layers.hpp"
#include "playback_clock.hpp"
#include "recording.hpp"

namespace playback2d {

struct AppState {
    // Files
    std::filesystem::path data_dir;
    std::vector<std::filesystem::path> files;
    std::filesystem::path loaded_file;
    std::optional<Recording> recording;
    std::string load_error;

    // Playback and view
    PlaybackClock clock;
    Camera camera;
    bool follow_ship{false};
    bool was_following{false};  // to zoom in when follow is switched on
    bool fit_requested{false};  // fit the whole track on the next frame
    ShipGeometry ship;
    std::vector<std::unique_ptr<Layer>> layers;

    // Loads a csv and rewinds playback. On failure keeps the current
    // recording and sets load_error.
    bool Load(const std::filesystem::path& path);
    void RefreshFiles();
    // Fit the camera to the whole track of the loaded recording
    void FitTrack();
};

}  // namespace playback2d
