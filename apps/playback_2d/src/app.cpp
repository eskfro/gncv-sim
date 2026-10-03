#include "app.hpp"

#include <SDL.h>

#include <algorithm>
#include <cstdio>
#include <string>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#include "app_state.hpp"
#include "panels.hpp"
#include "simdata/paths.hpp"
#include "simdata/run.hpp"

namespace playback2d {

namespace {

constexpr int kWindowWidth = 1400;
constexpr int kWindowHeight = 860;
constexpr float kSidePanelWidth = 330.0f;
constexpr float kPlaybackBarHeight = 70.0f;
constexpr double kMaxFrameDt = 0.25;  // [s] avoid jumps after a stall
constexpr Uint32 kMinFrameMs = 8;     // cap at ~120 fps when vsync is unavailable

void SetupState(AppState& state, const Options& options) {
    namespace fs = std::filesystem;
    playback::Session& session = state.session;
    const fs::path root = simdata::FindProjectRoot();
    session.SetProjectRoot(root);
    session.SetSimdataDir(simdata::SimdataDir(root));
    session.SetVesselOverride(options.vessel);

    // The input is a run (folder, metadata.json or csv), or a folder of runs
    fs::path run;
    std::error_code ec;
    if (!options.input.empty() && fs::is_directory(options.input, ec) && !simdata::IsRunDir(options.input)) {
        session.SetSimdataDir(options.input);
    } else {
        run = options.input;
    }

    state.layers = CreateDefaultLayers();
    session.clock.SetSpeed(options.speed);
    session.clock.SetLoop(options.loop);
    state.follow_ship = options.follow;
    session.Refresh();

    if (run.empty() && !session.Runs().empty()) run = session.Runs().front().data_file;  // newest
    if (!run.empty() && !state.Load(run)) {
        std::fprintf(stderr, "playback_2d: %s\n", session.Error().c_str());
    }
}

void DrawUi(AppState& state) {
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float side = std::min(kSidePanelWidth, display.x);
    const float main_w = std::max(display.x - side, 1.0f);
    const float view_h = std::max(display.y - kPlaybackBarHeight, 1.0f);

    // Viewport first so the panels draw on top of it
    DrawViewport(state, {{side, 0.0f}, {main_w, view_h}});
    DrawSidePanel(state, {{0.0f, 0.0f}, {side, display.y}});
    DrawPlaybackBar(state, {{side, view_h}, {main_w, kPlaybackBarHeight}});
    HandleShortcuts(state);
}

bool SaveScreenshot(SDL_Renderer* renderer, const std::filesystem::path& path) {
    int w = 0;
    int h = 0;
    if (SDL_GetRendererOutputSize(renderer, &w, &h) != 0) return false;
    SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormat(0, w, h, 32, SDL_PIXELFORMAT_ARGB8888);
    if (surface == nullptr) return false;
    const bool ok =
        SDL_RenderReadPixels(renderer, nullptr, SDL_PIXELFORMAT_ARGB8888, surface->pixels, surface->pitch) == 0 &&
        SDL_SaveBMP(surface, path.c_str()) == 0;
    SDL_FreeSurface(surface);
    return ok;
}

}  // namespace

int Run(const Options& options) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::fprintf(stderr, "playback_2d: SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    const auto window_flags = static_cast<SDL_WindowFlags>(SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Window* window = SDL_CreateWindow("playback_2d", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                          kWindowWidth, kWindowHeight, window_flags);
    if (window == nullptr) {
        std::fprintf(stderr, "playback_2d: SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
    if (renderer == nullptr) renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
    if (renderer == nullptr) {
        std::fprintf(stderr, "playback_2d: SDL_CreateRenderer failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;  // layout is fixed, nothing to save
    ImGui::StyleColorsDark();
    if (options.ui_scale != 1.0f) {
        ImGui::GetStyle().ScaleAllSizes(options.ui_scale);
        io.FontGlobalScale = options.ui_scale;
    }
    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    AppState state;
    SetupState(state, options);
    const bool screenshot_mode = !options.screenshot.empty();
    if (screenshot_mode) {
        state.session.clock.Pause();
        state.session.clock.Seek(options.screenshot_time);
    }

    int exit_code = 0;
    int frame_count = 0;
    std::string title;
    Uint64 last = SDL_GetPerformanceCounter();
    bool running = true;
    while (running) {
        const Uint32 frame_start = SDL_GetTicks();
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) running = false;
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE &&
                event.window.windowID == SDL_GetWindowID(window)) {
                running = false;
            }
            if (event.type == SDL_DROPFILE) {
                state.Load(event.drop.file);
                SDL_free(event.drop.file);
            }
        }
        if (SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED) {
            SDL_Delay(10);
            last = SDL_GetPerformanceCounter();
            continue;
        }

        const Uint64 now = SDL_GetPerformanceCounter();
        const double dt = static_cast<double>(now - last) / static_cast<double>(SDL_GetPerformanceFrequency());
        last = now;
        state.session.clock.Update(std::min(dt, kMaxFrameDt));

        const std::string wanted_title =
            state.session.Loaded() ? "playback_2d - " + state.session.GetRun().name : "playback_2d";
        if (wanted_title != title) {
            title = wanted_title;
            SDL_SetWindowTitle(window, title.c_str());
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        DrawUi(state);
        ImGui::Render();

        SDL_RenderSetScale(renderer, io.DisplayFramebufferScale.x, io.DisplayFramebufferScale.y);
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);

        // A few frames so ImGui has settled its layout before capturing
        if (screenshot_mode && ++frame_count == 3) {
            if (SaveScreenshot(renderer, options.screenshot)) {
                std::printf("Saved %s\n", options.screenshot.c_str());
            } else {
                std::fprintf(stderr, "playback_2d: screenshot failed: %s\n", SDL_GetError());
                exit_code = 1;
            }
            running = false;
        }
        SDL_RenderPresent(renderer);

        const Uint32 elapsed = SDL_GetTicks() - frame_start;
        if (elapsed < kMinFrameMs) SDL_Delay(kMinFrameMs - elapsed);
    }

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return exit_code;
}

}  // namespace playback2d
