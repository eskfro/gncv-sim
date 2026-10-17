#pragma once
// ============================================================================
// ImGui widgets shared by the playback apps: run list, run info, telemetry,
// playback bar and the playback keyboard shortcuts.
//
// They only draw into the current ImGui window, so each app keeps its own
// layout. Needs an ImGui context (link playback_ui).
// ============================================================================
#include <filesystem>
#include <functional>
#include <utility>
#include <vector>

#include "imgui.h"

#include "playback/session.hpp"

namespace playback {

struct Rect {
    ImVec2 pos;
    ImVec2 size;
};

// Window flags for panels laid out by the app
inline constexpr ImGuiWindowFlags kFixedWindow = ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                                                 ImGuiWindowFlags_NoCollapse |
                                                 ImGuiWindowFlags_NoSavedSettings;
void PlaceNextWindow(const Rect& rect);

using OpenRunFn = std::function<void(const std::filesystem::path&)>;

// Runs in the simdata folder, click to open. Shows load errors.
void RunsSection(Session& session, const OpenRunFn& open);
// Simulator, vessel, dt, ... of the loaded run, and its notes
void RunInfoSection(const Session& session);
// State, reference and actuators at the current playback time
void TelemetrySection(const Session& session);
// Transport buttons, speed and timeline, filling the current window
void PlaybackBar(Session& session);

// True when keys are not going to a text field or a dragged widget
bool ShortcutsAllowed();
// Space, arrows, 1, Home/End, L. Call when ShortcutsAllowed().
void HandlePlaybackKeys(PlaybackClock& clock);

using KeyHelp = std::vector<std::pair<const char*, const char*>>;
// Help for HandlePlaybackKeys(), to put first in an app's key list
const KeyHelp& PlaybackKeyHelp();
void KeysTable(const KeyHelp& keys);

// Heading as a compass bearing in [0, 360)
double CompassDeg(double rad);

}  // namespace playback
