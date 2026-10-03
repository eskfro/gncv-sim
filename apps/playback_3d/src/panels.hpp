#pragma once
// ============================================================================
// ImGui panels. Each one draws into a fixed screen rectangle laid out by App.
// ============================================================================
#include "app_state.hpp"
#include "playback/widgets.hpp"

namespace playback3d {

using playback::Rect;

// Runs, run info, telemetry, camera, layers and key help
void DrawSidePanel(AppState& state, const Rect& rect);
// Play/pause, speed and timeline
void DrawPlaybackBar(AppState& state, const Rect& rect);
// The 3D view: handles orbit, pan and zoom, builds state.scene for the
// renderer and draws the text on top of it. The window is transparent, the
// renderer fills it before ImGui is drawn.
void DrawViewport(AppState& state, const Rect& rect);
// Global keyboard shortcuts (ignored while typing in a widget)
void HandleShortcuts(AppState& state);

}  // namespace playback3d
