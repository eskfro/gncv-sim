#include "panels.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace playback2d {

namespace {

using playback::kFixedWindow;
using playback::PlaceNextWindow;

constexpr double kFollowShipPx = 140.0;  // hull length on screen when follow starts
constexpr ImU32 kSea = IM_COL32(16, 38, 58, 255);
constexpr ImU32 kOverlayText = IM_COL32(220, 230, 240, 220);

void ViewSection(AppState& state) {
    ImGui::Checkbox("Follow ship (F)", &state.follow_ship);
    ImGui::SameLine();
    if (ImGui::Button("Fit track (Z)")) {
        state.follow_ship = false;
        state.fit_requested = true;
    }
    ImGui::Spacing();
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
        {"Z, double click", "fit track"},
        {"Drag, wheel", "pan, zoom"},
        {"Drop run / csv", "open it"},
    });
    playback::KeysTable(keys);
}

// Little "N" arrow in the top right corner of the viewport
void NorthArrow(ImDrawList* dl, ImVec2 top_right) {
    const ImVec2 c{top_right.x - 30.0f, top_right.y + 40.0f};
    dl->AddTriangleFilled({c.x, c.y - 16.0f}, {c.x + 7.0f, c.y + 8.0f}, {c.x - 7.0f, c.y + 8.0f},
                          kOverlayText);
    dl->AddText({c.x - 4.0f, c.y + 10.0f}, kOverlayText, "N");
}

}  // namespace

void DrawSidePanel(AppState& state, const Rect& rect) {
    PlaceNextWindow(rect);
    ImGui::Begin("playback_2d", nullptr, kFixedWindow);
    if (ImGui::CollapsingHeader("Runs", ImGuiTreeNodeFlags_DefaultOpen)) {
        playback::RunsSection(state.session, [&state](const auto& path) { state.Load(path); });
    }
    if (ImGui::CollapsingHeader("Run info", ImGuiTreeNodeFlags_DefaultOpen)) playback::RunInfoSection(state.session);
    if (ImGui::CollapsingHeader("Telemetry", ImGuiTreeNodeFlags_DefaultOpen)) playback::TelemetrySection(state.session);
    if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) ViewSection(state);
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
                 kFixedWindow | ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoBringToFrontOnFocus |
                     ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar();

    const ImVec2 p0 = ImGui::GetCursorScreenPos();
    const ImVec2 size{std::max(ImGui::GetContentRegionAvail().x, 1.0f),
                      std::max(ImGui::GetContentRegionAvail().y, 1.0f)};
    const ImVec2 p1{p0.x + size.x, p0.y + size.y};
    ImGui::InvisibleButton("##canvas", size);
    const bool hovered = ImGui::IsItemHovered();
    const bool active = ImGui::IsItemActive();

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(p0, p1, kSea);

    if (!state.session.Loaded()) {
        const char* msg = "No run loaded.\nPick one on the left, or drop a run folder or csv here.";
        const ImVec2 ts = ImGui::CalcTextSize(msg);
        dl->AddText({p0.x + 0.5f * (size.x - ts.x), p0.y + 0.5f * (size.y - ts.y)}, kOverlayText, msg);
        ImGui::End();
        return;
    }

    Camera& cam = state.camera;
    cam.SetViewport(p0.x, p0.y, size.x, size.y);
    if (state.fit_requested) {
        state.FitTrack();
        state.fit_requested = false;
    }

    const Recording& recording = state.session.GetRecording();
    const playback::PlaybackClock& clock = state.session.clock;
    const Frame frame = recording.Sample(clock.Time());
    const NedPoint ship{frame.Get(col::kX), frame.Get(col::kY)};
    const ImGuiIO& io = ImGui::GetIO();

    if (active && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 0.0f)) {
        cam.Pan(io.MouseDelta.x, io.MouseDelta.y);
        state.follow_ship = false;
    }
    if (hovered && io.MouseWheel != 0.0f) {
        // While following, zoom about the ship so it stays centred
        const ScreenPoint anchor =
            state.follow_ship ? cam.ToScreen(ship) : ScreenPoint{io.MousePos.x, io.MousePos.y};
        cam.ZoomAt(std::pow(1.15, io.MouseWheel), anchor);
    }
    if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        state.follow_ship = false;
        state.FitTrack();
    }
    if (state.follow_ship && !state.was_following) {
        const double ship_px = state.ShipLength() * cam.PixelsPerMeter();
        if (ship_px < kFollowShipPx) cam.SetPixelsPerMeter(kFollowShipPx / state.ShipLength());
    }
    state.was_following = state.follow_ship;
    if (state.follow_ship) cam.CenterOn(ship);

    dl->PushClipRect(p0, p1, true);
    const DrawContext ctx{dl, cam, recording, frame, state.model, state.ShipLength()};
    for (auto& layer : state.layers) {
        if (layer->Enabled()) layer->Draw(ctx);
    }

    char text[256];
    std::snprintf(text, sizeof(text), "%s  (%s)\nt = %.1f s   %.2fx%s%s", state.session.GetRun().name.c_str(),
                  state.session.VesselName().c_str(), frame.Time(), clock.Speed(),
                  clock.Playing() ? "" : "   [paused]", clock.Loop() ? "   [loop]" : "");
    dl->AddText({p0.x + 60.0f, p0.y + 10.0f}, kOverlayText, text);
    NorthArrow(dl, {p1.x, p0.y});
    dl->PopClipRect();

    ImGui::End();
}

void HandleShortcuts(AppState& state) {
    if (!playback::ShortcutsAllowed() || !state.session.Loaded()) return;
    playback::HandlePlaybackKeys(state.session.clock);
    if (ImGui::IsKeyPressed(ImGuiKey_F, false)) state.follow_ship = !state.follow_ship;
    if (ImGui::IsKeyPressed(ImGuiKey_Z, false)) {
        state.follow_ship = false;
        state.fit_requested = true;
    }
}

}  // namespace playback2d
