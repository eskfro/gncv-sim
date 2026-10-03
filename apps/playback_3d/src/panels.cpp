#include "panels.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "imgui.h"

#include "simdata/channels.hpp"

namespace playback3d {

namespace {

namespace col = simdata::col;
using playback::kFixedWindow;
using playback::PlaceNextWindow;

constexpr ImU32 kOverlayText = IM_COL32(230, 236, 242, 230);
constexpr ImU32 kOverlayShadow = IM_COL32(0, 0, 0, 120);
constexpr double kZoomPerWheelStep = 1.15;

ImU32 ToIm(Rgba c) {
    auto channel = [](float v) { return static_cast<int>(std::lround(255.0f * std::clamp(v, 0.0f, 1.0f))); };
    return IM_COL32(channel(c.r), channel(c.g), channel(c.b), channel(c.a));
}

void ShadowText(ImDrawList* dl, ImVec2 pos, ImU32 color, const char* text) {
    dl->AddText({pos.x + 1.0f, pos.y + 1.0f}, kOverlayShadow, text);
    dl->AddText(pos, color, text);
}

void SetChase(AppState& state, bool chase) {
    if (chase && !state.chase && state.session.Loaded()) {
        const auto frame = state.session.GetRecording().Sample(state.session.clock.Time());
        state.chase_yaw = std::remainder(state.camera.Yaw() - frame.Get(col::kPsi), 2.0 * M_PI);
    }
    state.chase = chase;
}

void RequestFit(AppState& state) {
    state.follow_ship = false;
    state.chase = false;
    state.fit_requested = true;
}

void RequestReset(AppState& state) {
    state.follow_ship = true;
    state.reset_requested = true;
}

void CameraSection(AppState& state) {
    ImGui::Checkbox("Follow ship (F)", &state.follow_ship);
    ImGui::SameLine();
    bool chase = state.chase;
    if (ImGui::Checkbox("Chase heading (C)", &chase)) SetChase(state, chase);
    if (ImGui::Button("Fit track (Z)")) RequestFit(state);
    ImGui::SameLine();
    if (ImGui::Button("Reset view (R)")) RequestReset(state);

    ImGui::PushItemWidth(-120.0f);
    float attitude = static_cast<float>(state.motion.attitude);
    if (ImGui::SliderFloat("Roll/pitch scale", &attitude, 1.0f, 20.0f, "%.1fx", ImGuiSliderFlags_Logarithmic)) {
        state.motion.attitude = attitude;
    }
    ImGui::SetItemTooltip("Exaggerate roll and pitch, which are often too small to see");
    float heave = static_cast<float>(state.motion.heave);
    if (ImGui::SliderFloat("Heave scale", &heave, 1.0f, 20.0f, "%.1fx", ImGuiSliderFlags_Logarithmic)) {
        state.motion.heave = heave;
    }
    ImGui::PopItemWidth();
}

void LayersSection(AppState& state) {
    ImGui::PushItemWidth(-120.0f);  // leave room for the labels
    for (auto& layer : state.layers) {
        ImGui::PushID(layer.get());
        ImGui::Checkbox(layer->Name().c_str(), &layer->Enabled());
        if (layer->Enabled()) {
            ImGui::Indent();
            layer->DrawSettings();
            ImGui::Unindent();
        }
        ImGui::PopID();
    }
    ImGui::PopItemWidth();
}

void KeysSection() {
    playback::KeyHelp keys = playback::PlaybackKeyHelp();
    keys.insert(keys.end(), {
        {"F", "follow ship"},
        {"C", "chase heading"},
        {"Z, double click", "fit track"},
        {"R", "reset view"},
        {"Drag", "orbit"},
        {"Right / Shift drag", "pan"},
        {"Wheel", "zoom"},
        {"Drop run / csv", "open it"},
    });
    playback::KeysTable(keys);
}

// Compass in the top right corner: the arrow points north as seen from the camera
void Compass(ImDrawList* dl, ImVec2 top_right, double yaw) {
    const ImVec2 c{top_right.x - 40.0f, top_right.y + 44.0f};
    constexpr float r = 22.0f;
    // Screen direction of north: right = (-sin yaw), up = cos yaw
    const ImVec2 n{static_cast<float>(-std::sin(yaw)), static_cast<float>(-std::cos(yaw))};
    const ImVec2 side{-n.y, n.x};
    dl->AddCircle(c, r, IM_COL32(230, 236, 242, 90), 32, 1.5f);
    dl->AddTriangleFilled({c.x + n.x * r, c.y + n.y * r}, {c.x + side.x * 6.0f, c.y + side.y * 6.0f},
                          {c.x - side.x * 6.0f, c.y - side.y * 6.0f}, IM_COL32(235, 90, 70, 230));
    dl->AddTriangleFilled({c.x - n.x * r, c.y - n.y * r}, {c.x - side.x * 6.0f, c.y - side.y * 6.0f},
                          {c.x + side.x * 6.0f, c.y + side.y * 6.0f}, IM_COL32(230, 236, 242, 160));
    const ImVec2 label{c.x + n.x * (r + 10.0f) - 4.0f, c.y + n.y * (r + 10.0f) - 7.0f};
    ShadowText(dl, label, kOverlayText, "N");
}

}  // namespace

void DrawSidePanel(AppState& state, const Rect& rect) {
    PlaceNextWindow(rect);
    ImGui::Begin("playback_3d", nullptr, kFixedWindow);
    if (ImGui::CollapsingHeader("Runs", ImGuiTreeNodeFlags_DefaultOpen)) {
        playback::RunsSection(state.session, [&state](const auto& path) { state.Load(path); });
    }
    if (ImGui::CollapsingHeader("Run info", ImGuiTreeNodeFlags_DefaultOpen)) playback::RunInfoSection(state.session);
    if (ImGui::CollapsingHeader("Telemetry", ImGuiTreeNodeFlags_DefaultOpen)) playback::TelemetrySection(state.session);
    if (ImGui::CollapsingHeader("Camera", ImGuiTreeNodeFlags_DefaultOpen)) CameraSection(state);
    if (ImGui::CollapsingHeader("Layers", ImGuiTreeNodeFlags_DefaultOpen)) LayersSection(state);
    if (ImGui::CollapsingHeader("Keys")) KeysSection();
    ImGui::End();
}

void DrawPlaybackBar(AppState& state, const Rect& rect) {
    PlaceNextWindow(rect);
    ImGui::Begin("##playback", nullptr, kFixedWindow | ImGuiWindowFlags_NoTitleBar);
    playback::PlaybackBar(state.session);
    ImGui::End();
}

void DrawViewport(AppState& state, const Rect& rect) {
    PlaceNextWindow(rect);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {0.0f, 0.0f});
    ImGui::Begin("##viewport", nullptr,
                 kFixedWindow | ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBackground |
                     ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();

    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 size{std::max(ImGui::GetContentRegionAvail().x, 1.0f),
                      std::max(ImGui::GetContentRegionAvail().y, 1.0f)};
    const ImVec2 p1{p0.x + size.x, p0.y + size.y};
    ImGui::InvisibleButton("##canvas", size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();
    state.viewport = {p0, size};

    ImDrawList* dl = ImGui::GetWindowDrawList();
    if (!state.session.Loaded()) {
        state.scene_ready = false;
        const char* msg = "No run loaded.\nPick one on the left, or drop a run folder or csv here.";
        const ImVec2 ts = ImGui::CalcTextSize(msg);
        ShadowText(dl, {p0.x + 0.5f * (size.x - ts.x), p0.y + 0.5f * (size.y - ts.y)}, kOverlayText, msg);
        ImGui::End();
        return;
    }

    OrbitCamera& cam = state.camera;
    cam.SetViewport(p0.x, p0.y, size.x, size.y);
    if (state.reset_requested) {
        state.ResetView();
        state.reset_requested = false;
    }
    if (state.fit_requested) {
        state.FitTrack();
        state.fit_requested = false;
    }

    const auto& recording = state.session.GetRecording();
    const auto& clock = state.session.clock;
    const simdata::Frame frame = recording.Sample(clock.Time());
    const double psi = frame.Get(col::kPsi);
    const ImGuiIO& io = ImGui::GetIO();

    // Camera: chase heading, then mouse input, then follow
    if (state.chase) cam.SetYaw(psi + state.chase_yaw);
    const bool left_drag = active && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.0f);
    const bool right_drag = active && ImGui::IsMouseDragging(ImGuiMouseButton_Right, 0.0f);
    if (right_drag || (left_drag && io.KeyShift)) {
        cam.Pan(io.MouseDelta.x, io.MouseDelta.y);
        state.follow_ship = false;
    } else if (left_drag) {
        cam.Orbit(io.MouseDelta.x, io.MouseDelta.y);
    }
    if (hovered && io.MouseWheel != 0.0f) cam.Zoom(std::pow(kZoomPerWheelStep, io.MouseWheel));
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        RequestFit(state);
        state.FitTrack();
        state.fit_requested = false;
    }
    if (state.chase) state.chase_yaw = std::remainder(cam.Yaw() - psi, 2.0 * M_PI);
    if (state.follow_ship) cam.SetTarget({frame.Get(col::kX), frame.Get(col::kY), 0.0});

    // Scene for the renderer, around the camera target
    state.scene.Clear(cam.Target());
    const SceneContext ctx{recording,  frame,         cam, state.model, state.ShipLength(),
                           state.motion, cam.MetersPerPixel()};
    for (auto& layer : state.layers) {
        if (layer->Enabled()) layer->Build(ctx, state.scene);
    }
    state.scene_ready = true;

    // Text on top of the 3D view
    dl->PushClipRect(p0, p1, true);
    for (const auto& label : state.scene.Labels()) {
        const auto s = cam.Project(label.world);
        if (!s || s->x < p0.x || s->x > p1.x || s->y < p0.y || s->y > p1.y) continue;
        ShadowText(dl, {static_cast<float>(s->x) + 5.0f, static_cast<float>(s->y) + 2.0f}, ToIm(label.color),
                   label.text.c_str());
    }
    char text[256];
    std::snprintf(text, sizeof(text), "%s  (%s)\nt = %.1f s   %.2fx%s%s\ncamera %.0f m%s%s",
                  state.session.GetRun().name.c_str(), state.session.VesselName().c_str(), frame.Time(),
                  clock.Speed(), clock.Playing() ? "" : "   [paused]", clock.Loop() ? "   [loop]" : "",
                  cam.Distance(), state.follow_ship ? "   [follow]" : "", state.chase ? "   [chase]" : "");
    ShadowText(dl, {p0.x + 12.0f, p0.y + 10.0f}, kOverlayText, text);
    Compass(dl, {p1.x, p0.y}, cam.Yaw());
    dl->PopClipRect();

    ImGui::End();
}

void HandleShortcuts(AppState& state) {
    if (!playback::ShortcutsAllowed() || !state.session.Loaded()) return;
    playback::HandlePlaybackKeys(state.session.clock);
    if (ImGui::IsKeyPressed(ImGuiKey_F, false)) state.follow_ship = !state.follow_ship;
    if (ImGui::IsKeyPressed(ImGuiKey_C, false)) SetChase(state, !state.chase);
    if (ImGui::IsKeyPressed(ImGuiKey_Z, false)) RequestFit(state);
    if (ImGui::IsKeyPressed(ImGuiKey_R, false)) RequestReset(state);
}

}  // namespace playback3d
