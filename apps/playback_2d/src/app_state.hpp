#pragma once
// ============================================================================
// Everything the UI panels read and change. Owned by App.
// ============================================================================
#include <filesystem>
#include <memory>
#include <vector>

#include "camera.hpp"
#include "layers.hpp"
#include "playback/session.hpp"
#include "vessel_model/model_2d.hpp"

namespace playback2d {

struct AppState {
    // Runs, the loaded recording, its vessel and the playback clock
    playback::Session session;

    // View
    Camera camera;
    bool follow_ship{false};
    bool was_following{false};  // to zoom in when follow is switched on
    bool fit_requested{false};  // fit the whole track on the next frame
    vessel_model::Model2D model;  // top view of the loaded run's vessel
    std::vector<std::unique_ptr<Layer>> layers;

    // Opens a run (folder, metadata.json or csv), loads its vessel's
    // model_2d.svg and fits the view. On failure keeps the current run and
    // session.Error() says why.
    bool Load(const std::filesystem::path& path);
    // Fit the camera to the whole track of the loaded run
    void FitTrack();
    double ShipLength() const;

private:
    void LoadModel();
};

}  // namespace playback2d
