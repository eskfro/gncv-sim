#include "panels.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace playback2d {

namespace {

constexpr double kRad2Deg = 180.0 / M_PI;
constexpr double kFollowShipPx = 140.0;  // hull length on screen when follow starts
constexpr ImU32 kSea = IM_COL32(16, 38, 58, 255);
constexpr ImU32 kOverlayText = IM_COL32(220, 230, 240, 220);
constexpr ImGuiWindowFlags kFixedWindow = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                                          ImGuiWindowFlags_NoCollapse |
                                          ImGuiWindowFlags_NoSavedSettings;

void PlaceNextWindow(const Rect& rect) {
    ImGui::SetNextWindowPos(rect.pos);
    ImGui::SetNextWindowSize(rect.size);
}

// Heading as a compass bearing in [0, 360)
double CompassDeg(double rad) {
    const double deg = std::fmod(rad * kRad2Deg, 360.0);
    return deg < 0.0 ? deg + 360.0 : deg;
}

void TelemetryRow(const char* name, const char* fmt, double value) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name);
    ImGui::TableNextColumn();
    ImGui::Text(fmt, value);
}

void FilesSection(AppState& state) {
    ImGui::TextDisabled("%s", state.data_dir.c_str());
    if (ImGui::Button("Refresh")) state.RefreshFiles();
    ImGui::SameLine();
    ImGui::TextDisabled("%zu file(s)", state.files.size());

    if (ImGui::BeginListBox("##files", {-FLT_MIN, 8.5f * ImGui::GetTextLineHeightWithSpacing()})) {
        for (const auto& file : state.files) {
            const bool selected = file == state.loaded_file;
            if (ImGui::Selectable(file.filename().c_str(), selected)) state.Load(file);
            if (selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndListBox();
    }
    if (state.files.empty()) {
        ImGui::TextWrapped("No csv files here. Run simulator_v1 (make v1), or drop a csv onto the window.");
    }
    if (!state.load_error.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 120, 110, 255));
        ImGui::TextWrapped("%s", state.load_error.c_str());
        ImGui::PopStyleColor();
    }
    if (state.recording && state.recording->SkippedRows() > 0) {
        ImGui::TextDisabled("%zu malformed row(s) skipped", state.recording->SkippedRows());
    }
}

void TelemetrySection(const AppState& state) {
    if (!state.recording) {
        ImGui::TextDisabled("No recording loaded");
        return;
    }
    const Frame f = state.recording->Sample(state.clock.Time());
    if (!ImGui::BeginTable("##telemetry", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        return;
    }

    const double u = f.Get(col::kU);
    const double v = f.Get(col::kV);
    TelemetryRow("t", "%.2f s", f.Time());
    TelemetryRow("North x", "%.1f m", f.Get(col::kX));
    TelemetryRow("East y", "%.1f m", f.Get(col::kY));
    TelemetryRow("Heading psi", "%.1f deg", CompassDeg(f.Get(col::kPsi)));
    TelemetryRow("Speed U", "%.2f m/s", std::hypot(u, v));
    TelemetryRow("Surge u", "%.2f m/s", u);
    TelemetryRow("Sway v", "%.2f m/s", v);
    TelemetryRow("Yaw rate r", "%.2f deg/s", f.Get(col::kR) * kRad2Deg);
    TelemetryRow("Roll phi", "%.2f deg", f.Get(col::kPhi) * kRad2Deg);

    if (f.Has(col::kGuidanceMode)) {
        const int mode = static_cast<int>(f.Get(col::kGuidanceMode));
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted("Guidance");
        ImGui::TableNextColumn();
        const bool known = mode >= 0 && mode < static_cast<int>(kGuidanceModes.size());
        ImGui::TextUnformatted(known ? kGuidanceModes[mode] : "?");
    }
    if (f.Has(col::kPsiD)) TelemetryRow("Heading ref psi_d", "%.1f deg", CompassDeg(f.Get(col::kPsiD)));
    if (f.Has(col::kUd)) TelemetryRow("Speed ref u_d", "%.2f m/s", f.Get(col::kUd));
    if (f.Has(col::kDeltaR)) TelemetryRow("Rudder delta_r", "%.1f deg", f.Get(col::kDeltaR) * kRad2Deg);
    if (f.Has(col::kNmp)) TelemetryRow("Main prop n_mp", "%.0f rpm", f.Get(col::kNmp));
    if (f.Has(col::kNtt)) TelemetryRow("Tunnel n_tt", "%.0f rpm", f.Get(col::kNtt));
    ImGui::EndTable();
}

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
    const char* keys[][2] = {
        {"Space", "play / pause"},       {"Left / Right", "seek -/+ 5 s (Shift: 1 s)"},
        {"Up / Down", "faster / slower"}, {"1", "speed 1x"},
        {"Home / End", "start / end"},   {"L", "loop on/off"},
        {"F", "follow ship"},            {"Z, double click", "fit track"},
        {"Drag, wheel", "pan, zoom"},    {"Drop csv", "open file"},
    };
    if (ImGui::BeginTable("##keys", 2, ImGuiTableFlags_SizingStretchProp)) {
        for (const auto& k : keys) {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextDisabled("%s", k[0]);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted(k[1]);
        }
        ImGui::EndTable();
    }
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
    ImGui::Begin("playback-2d", nullptr, kFixedWindow);
    if (ImGui::CollapsingHeader("Files", ImGuiTreeNodeFlags_DefaultOpen)) FilesSection(state);
    if (ImGui::CollapsingHeader("Telemetry", ImGuiTreeNodeFlags_DefaultOpen)) TelemetrySection(state);
    if (ImGui::CollapsingHeader("View", ImGuiTreeNodeFlags_DefaultOpen)) ViewSection(state);
    if (ImGui::CollapsingHeader("Keys")) KeysSection();
    ImGui::End();
}

void DrawPlaybackBar(AppState& state, const Rect& rect) {
    PlaybackClock& clock = state.clock;
    PlaceNextWindow(rect);
    ImGui::Begin("##playback", nullptr, kFixedWindow | ImGuiWindowFlags_NoTitleBar);
    ImGui::BeginDisabled(!state.recording);

    // Transport
    if (ImGui::Button("|<")) clock.Seek(clock.Start());
    ImGui::SameLine();
    if (ImGui::Button("-5s")) clock.SeekBy(-5.0);
    ImGui::SameLine();
    if (ImGui::Button(clock.Playing() ? "Pause" : " Play ", {60.0f, 0.0f})) clock.TogglePlay();
    ImGui::SameLine();
    if (ImGui::Button("+5s")) clock.SeekBy(5.0);
    ImGui::SameLine();
    if (ImGui::Button(">|")) clock.Seek(clock.End());
    ImGui::SameLine();
    bool loop = clock.Loop();
    if (ImGui::Checkbox("Loop", &loop)) clock.SetLoop(loop);

    // Speed
    ImGui::SameLine(0.0f, 30.0f);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Speed");
    ImGui::SameLine();
    if (ImGui::Button("-")) clock.Slower();
    ImGui::SameLine();
    float speed = static_cast<float>(clock.Speed());
    ImGui::SetNextItemWidth(160.0f);
    if (ImGui::SliderFloat("##speed", &speed, static_cast<float>(PlaybackClock::kMinSpeed),
                           static_cast<float>(PlaybackClock::kMaxSpeed), "%.2fx",
                           ImGuiSliderFlags_Logarithmic)) {
        clock.SetSpeed(speed);
    }
    ImGui::SameLine();
    if (ImGui::Button("+")) clock.Faster();
    ImGui::SameLine();
    if (ImGui::Button("1x")) clock.SetSpeed(1.0);

    // Timeline
    double t = clock.Time();
    const double t_min = clock.Start();
    const double t_max = clock.End();
    char label[64];
    std::snprintf(label, sizeof(label), "%.1f / %.1f s", t, t_max);
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::SliderScalar("##time", ImGuiDataType_Double, &t, &t_min, &t_max, label)) clock.Seek(t);

    ImGui::EndDisabled();
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

    if (!state.recording) {
        const char* msg = "No recording loaded.\nPick a file on the left, or drop a csv here.";
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

    const Frame frame = state.recording->Sample(state.clock.Time());
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
        const double ship_px = state.ship.length * cam.PixelsPerMeter();
        if (ship_px < kFollowShipPx) cam.SetPixelsPerMeter(kFollowShipPx / state.ship.length);
    }
    state.was_following = state.follow_ship;
    if (state.follow_ship) cam.CenterOn(ship);

    dl->PushClipRect(p0, p1, true);
    const DrawContext ctx{dl, cam, *state.recording, frame, state.ship};
    for (auto& layer : state.layers) {
        if (layer->Enabled()) layer->Draw(ctx);
    }

    char text[160];
    std::snprintf(text, sizeof(text), "%s\nt = %.1f s   %.2fx%s%s", state.recording->SourceName().c_str(),
                  frame.Time(), state.clock.Speed(), state.clock.Playing() ? "" : "   [paused]",
                  state.clock.Loop() ? "   [loop]" : "");
    dl->AddText({p0.x + 60.0f, p0.y + 10.0f}, kOverlayText, text);
    NorthArrow(dl, {p1.x, p0.y});
    dl->PopClipRect();

    ImGui::End();
}

void HandleShortcuts(AppState& state) {
    const ImGuiIO& io = ImGui::GetIO();
    if (io.WantTextInput || ImGui::IsAnyItemActive() || !state.recording) return;

    PlaybackClock& clock = state.clock;
    const double seek = io.KeyShift ? 1.0 : 5.0;
    if (ImGui::IsKeyPressed(ImGuiKey_Space, false)) clock.TogglePlay();
    if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow)) clock.SeekBy(-seek);
    if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) clock.SeekBy(seek);
    if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) clock.Faster();
    if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) clock.Slower();
    if (ImGui::IsKeyPressed(ImGuiKey_1, false)) clock.SetSpeed(1.0);
    if (ImGui::IsKeyPressed(ImGuiKey_Home, false)) clock.Seek(clock.Start());
    if (ImGui::IsKeyPressed(ImGuiKey_End, false)) clock.Seek(clock.End());
    if (ImGui::IsKeyPressed(ImGuiKey_L, false)) clock.SetLoop(!clock.Loop());
    if (ImGui::IsKeyPressed(ImGuiKey_F, false)) state.follow_ship = !state.follow_ship;
    if (ImGui::IsKeyPressed(ImGuiKey_Z, false)) {
        state.follow_ship = false;
        state.fit_requested = true;
    }
}

}  // namespace playback2d
