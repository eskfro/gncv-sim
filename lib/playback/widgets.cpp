#include "playback/widgets.hpp"

#include <cmath>
#include <cstdio>
#include <string>

#include "simdata/channels.hpp"

namespace playback {

namespace col = simdata::col;

namespace {

constexpr double kRad2Deg = 180.0 / M_PI;
constexpr ImU32 kErrorText = IM_COL32(255, 120, 110, 255);
constexpr ImU32 kNoteText = IM_COL32(240, 200, 110, 255);

void Row(const char* name, const char* fmt, double value) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name);
    ImGui::TableNextColumn();
    ImGui::Text(fmt, value);
}

void TextRow(const char* name, const std::string& value) {
    ImGui::TableNextRow();
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(name);
    ImGui::TableNextColumn();
    ImGui::TextUnformatted(value.c_str());
}

}  // namespace

void PlaceNextWindow(const Rect& rect) {
    ImGui::SetNextWindowPos(rect.pos);
    ImGui::SetNextWindowSize(rect.size);
}

double CompassDeg(double rad) {
    const double deg = std::fmod(rad * kRad2Deg, 360.0);
    return deg < 0.0 ? deg + 360.0 : deg;
}

void RunsSection(Session& session, const OpenRunFn& open) {
    ImGui::TextDisabled("%s", session.SimdataDir().c_str());
    if (ImGui::Button("Refresh")) session.Refresh();
    ImGui::SameLine();
    ImGui::TextDisabled("%zu run(s)", session.Runs().size());

    if (ImGui::BeginListBox("##runs", {-FLT_MIN, 8.5f * ImGui::GetTextLineHeightWithSpacing()})) {
        for (const auto& run : session.Runs()) {
            const bool selected = session.IsCurrent(run);
            ImGui::PushID(run.data_file.c_str());
            if (!run.HasMetadata()) ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
            if (ImGui::Selectable(run.name.c_str(), selected)) open(run.data_file);
            if (!run.HasMetadata()) {
                ImGui::PopStyleColor();
                ImGui::SetItemTooltip("No metadata.json (run from before run folders)");
            }
            if (selected) ImGui::SetItemDefaultFocus();
            ImGui::PopID();
        }
        ImGui::EndListBox();
    }
    if (session.Runs().empty()) {
        ImGui::TextWrapped("No runs here. Run simulator_v1 (make v1), or drop a run folder or csv onto the window.");
    }
    if (!session.Error().empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, kErrorText);
        ImGui::TextWrapped("%s", session.Error().c_str());
        ImGui::PopStyleColor();
    }
}

void RunInfoSection(const Session& session) {
    if (!session.Loaded()) {
        ImGui::TextDisabled("No run loaded");
        return;
    }
    if (ImGui::BeginTable("##runinfo", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        TextRow("Run", session.GetRun().name);
        TextRow("Vessel", session.VesselName());
        if (const auto* m = session.GetMetadata()) {
            if (!m->simulator.empty()) TextRow("Simulator", m->simulator);
            if (!m->created.empty()) TextRow("Created", m->created);
            if (m->dt > 0.0) Row("Time step", "%g s", m->dt);
        }
        const auto& rec = session.GetRecording();
        Row("Duration", "%.1f s", rec.Duration());
        TextRow("Samples", std::to_string(rec.Rows()));
        ImGui::EndTable();
    }
    for (const auto& note : session.Notes()) {
        ImGui::PushStyleColor(ImGuiCol_Text, kNoteText);
        ImGui::TextWrapped("%s", note.c_str());
        ImGui::PopStyleColor();
    }
}

void TelemetrySection(const Session& session) {
    if (!session.Loaded()) {
        ImGui::TextDisabled("No run loaded");
        return;
    }
    const simdata::Frame f = session.GetRecording().Sample(session.clock.Time());
    if (!ImGui::BeginTable("##telemetry", 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
        return;
    }

    const double u = f.Get(col::kU);
    const double v = f.Get(col::kV);
    const double w = f.Get(col::kW);
    Row("t", "%.2f s", f.Time());
    Row("North x", "%.1f m", f.Get(col::kX));
    Row("East y", "%.1f m", f.Get(col::kY));
    if (f.Has(col::kZ)) Row("Down z", "%.2f m", f.Get(col::kZ));
    Row("Heading psi", "%.1f deg", CompassDeg(f.Get(col::kPsi)));
    if (f.Has(col::kPhi)) Row("Roll phi", "%.2f deg", f.Get(col::kPhi) * kRad2Deg);
    if (f.Has(col::kTheta)) Row("Pitch theta", "%.2f deg", f.Get(col::kTheta) * kRad2Deg);
    Row("Speed U", "%.2f m/s", std::sqrt(u * u + v * v + w * w));
    Row("Surge u", "%.2f m/s", u);
    Row("Sway v", "%.2f m/s", v);
    if (f.Has(col::kR)) Row("Yaw rate r", "%.2f deg/s", f.Get(col::kR) * kRad2Deg);

    if (f.Has(col::kGuidanceMode)) {
        const auto modes = session.GuidanceModes();
        const int mode = static_cast<int>(f.Get(col::kGuidanceMode));
        const bool known = mode >= 0 && mode < static_cast<int>(modes.size());
        TextRow("Guidance", known ? modes[mode] : "? (" + std::to_string(mode) + ")");
    }
    if (f.Has(col::kPsiD)) Row("Heading ref psi_d", "%.1f deg", CompassDeg(f.Get(col::kPsiD)));
    if (f.Has(col::kUd)) Row("Speed ref u_d", "%.2f m/s", f.Get(col::kUd));
    if (f.Has(col::kDeltaR)) Row("Rudder delta_r", "%.1f deg", f.Get(col::kDeltaR) * kRad2Deg);
    if (f.Has(col::kNmp)) Row("Main prop n_mp", "%.0f rpm", f.Get(col::kNmp));
    if (f.Has(col::kNtt)) Row("Tunnel n_tt", "%.0f rpm", f.Get(col::kNtt));
    ImGui::EndTable();
}

void PlaybackBar(Session& session) {
    PlaybackClock& clock = session.clock;
    ImGui::BeginDisabled(!session.Loaded());

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
                           static_cast<float>(PlaybackClock::kMaxSpeed), "%.2fx", ImGuiSliderFlags_Logarithmic)) {
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
}

bool ShortcutsAllowed() {
    const ImGuiIO& io = ImGui::GetIO();
    return !io.WantTextInput && !ImGui::IsAnyItemActive();
}

void HandlePlaybackKeys(PlaybackClock& clock) {
    const double seek = ImGui::GetIO().KeyShift ? 1.0 : 5.0;
    if (ImGui::IsKeyPressed(ImGuiKey_Space, false)) clock.TogglePlay();
    if (ImGui::IsKeyPressed(ImGuiKey_LeftArrow)) clock.SeekBy(-seek);
    if (ImGui::IsKeyPressed(ImGuiKey_RightArrow)) clock.SeekBy(seek);
    if (ImGui::IsKeyPressed(ImGuiKey_UpArrow)) clock.Faster();
    if (ImGui::IsKeyPressed(ImGuiKey_DownArrow)) clock.Slower();
    if (ImGui::IsKeyPressed(ImGuiKey_1, false)) clock.SetSpeed(1.0);
    if (ImGui::IsKeyPressed(ImGuiKey_Home, false)) clock.Seek(clock.Start());
    if (ImGui::IsKeyPressed(ImGuiKey_End, false)) clock.Seek(clock.End());
    if (ImGui::IsKeyPressed(ImGuiKey_L, false)) clock.SetLoop(!clock.Loop());
}

const KeyHelp& PlaybackKeyHelp() {
    static const KeyHelp keys = {
        {"Space", "play / pause"},       {"Left / Right", "seek -/+ 5 s (Shift: 1 s)"},
        {"Up / Down", "faster / slower"}, {"1", "speed 1x"},
        {"Home / End", "start / end"},   {"L", "loop on/off"},
    };
    return keys;
}

void KeysTable(const KeyHelp& keys) {
    if (!ImGui::BeginTable("##keys", 2, ImGuiTableFlags_SizingStretchProp)) return;
    for (const auto& [key, action] : keys) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextDisabled("%s", key);
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(action);
    }
    ImGui::EndTable();
}

}  // namespace playback
