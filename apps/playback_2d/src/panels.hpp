#pragma once
// ============================================================================
// ImGui panels. Each one draws into a fixed screen rectangle laid out by App.
// ============================================================================
#include "imgui.h"

#include "app_state.hpp"
#include "playback/widgets.hpp"

namespace playback2d {

using playback::Rect;

// Runs, run info, telemetry, layers and key help
void DrawSidePanel(AppState& state, const Rect& rect);
// Play/pause, speed and timeline
void DrawPlaybackBar(AppState& state, const Rect& rect);
// The 2D scene. Handles pan (drag), zoom (wheel) and fit (double click).
void DrawViewport(AppState& state, const Rect& rect);
// Global keyboard shortcuts (ignored while typing in a widget)
void HandleShortcuts(AppState& state);

}  // namespace playback2d
