#pragma once
// ============================================================================
// Scene layers of the 3D view, built back to front each frame.
//
// To draw something new, subclass Layer and add it in CreateDefaultLayers().
// Each layer can be toggled and can show its own settings in the View panel.
// Flat things on the water (track, arrows, markers) are sized in pixels via
// ctx.meters_per_pixel, so they keep their size on screen when zooming.
// ============================================================================
#include <memory>
#include <string>
#include <vector>

#include "orbit_camera.hpp"
#include "scene.hpp"
#include "simdata/recording.hpp"
#include "vessel_model/model_3d.hpp"
#include "vessel_pose.hpp"

namespace playback3d {

struct SceneContext {
    const simdata::Recording& recording;
    const simdata::Frame& frame;  // the recording sampled at the current playback time
    const OrbitCamera& camera;
    const vessel_model::Model3D& model;  // the vessel, body frame
    double ship_length;                  // [m]
    MotionScale motion;                  // exaggeration of roll, pitch and heave
    double meters_per_pixel;             // at the camera target
};

class Layer {
public:
    Layer(std::string name, bool enabled) : name_(std::move(name)), enabled_(enabled) {}
    virtual ~Layer() = default;

    virtual void Build(const SceneContext& ctx, SceneBatch& out) = 0;
    // ImGui widgets for this layer's options (shown under its checkbox)
    virtual void DrawSettings() {}

    const std::string& Name() const { return name_; }
    bool& Enabled() { return enabled_; }

private:
    std::string name_;
    bool enabled_;
};

std::vector<std::unique_ptr<Layer>> CreateDefaultLayers();

}  // namespace playback3d
