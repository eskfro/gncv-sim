#pragma once
// ============================================================================
// Everything the UI panels read and change. Owned by App.
// ============================================================================
#include <cstdint>
#include <filesystem>
#include <memory>
#include <vector>

#include "layers.hpp"
#include "orbit_camera.hpp"
#include "playback/session.hpp"
#include "playback/widgets.hpp"
#include "scene.hpp"
#include "vessel_model/model_3d.hpp"
#include "vessel_pose.hpp"

namespace playback3d {

struct AppState {
    // Runs, the loaded recording, its vessel and the playback clock
    playback::Session session;

    // The loaded run's vessel. model_version changes when it is replaced,
    // so the renderer knows to upload it again.
    vessel_model::Model3D model;
    std::uint64_t model_version{0};

    // Camera
    OrbitCamera camera;
    bool follow_ship{true};   // keep the camera target on the ship
    bool chase{false};        // turn the camera with the ship's heading
    double chase_yaw{0.0};    // camera yaw relative to the heading while chasing
    bool fit_requested{false};
    bool reset_requested{false};

    // Drawing
    MotionScale motion;
    std::vector<std::unique_ptr<Layer>> layers;

    // Built by the viewport each frame, drawn by the renderer
    SceneBatch scene;
    playback::Rect viewport{};
    bool scene_ready{false};

    // Opens a run (folder, metadata.json or csv), loads its vessel's
    // model_3d.obj and resets the view. On failure keeps the current run and
    // session.Error() says why.
    bool Load(const std::filesystem::path& path);
    // Look at the whole track from above
    void FitTrack();
    // Close behind the ship, at the current time
    void ResetView();
    double ShipLength() const;

private:
    void LoadModel();
};

}  // namespace playback3d
