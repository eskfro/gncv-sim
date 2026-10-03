#pragma once
// ============================================================================
// Scene layers drawn in the 2D view, back to front.
//
// To draw something new, subclass Layer and add it in CreateDefaultLayers().
// Each layer can be toggled and can show its own settings in the Layers panel.
// ============================================================================
#include <memory>
#include <string>
#include <vector>

#include "imgui.h"

#include "camera.hpp"
#include "simdata/channels.hpp"
#include "simdata/recording.hpp"
#include "vessel_model/model_2d.hpp"

namespace playback2d {

using simdata::Frame;
using simdata::Recording;
namespace col = simdata::col;

struct DrawContext {
    ImDrawList* draw_list;
    const Camera& camera;
    const Recording& recording;
    const Frame& frame;  // the recording sampled at the current playback time
    const vessel_model::Model2D& model;  // top view of the vessel, body frame
    double ship_length;  // [m]
};

class Layer {
public:
    Layer(std::string name, bool enabled) : name_(std::move(name)), enabled_(enabled) {}
    virtual ~Layer() = default;

    virtual void Draw(const DrawContext& ctx) = 0;
    // ImGui widgets for this layer's options (shown under its checkbox)
    virtual void DrawSettings() {}

    const std::string& Name() const { return name_; }
    bool& Enabled() { return enabled_; }

private:
    std::string name_;
    bool enabled_;
};

std::vector<std::unique_ptr<Layer>> CreateDefaultLayers();

}  // namespace playback2d
