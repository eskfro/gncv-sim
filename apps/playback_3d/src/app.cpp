#include "app.hpp"

#include <SDL.h>
// Linux: libGL exports the OpenGL 3.3 functions, so no loader is needed
#define GL_GLEXT_PROTOTYPES 1
#include <SDL_opengl.h>

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "imgui_impl_sdl2.h"

#include "app_state.hpp"
#include "panels.hpp"
#include "renderer.hpp"
#include "simdata/paths.hpp"
#include "simdata/run.hpp"

namespace playback3d {

namespace {

constexpr int kWindowWidth = 1400;
constexpr int kWindowHeight = 860;
constexpr float kSidePanelWidth = 340.0f;
constexpr float kPlaybackBarHeight = 70.0f;
constexpr double kMaxFrameDt = 0.25;  // [s] avoid jumps after a stall
constexpr Uint32 kMinFrameMs = 8;     // cap at ~120 fps when vsync is unavailable
constexpr Rgba kSky = Rgb8(148, 176, 200);
constexpr double kFogStart = 8.0;     // [camera distances]
constexpr double kFogEnd = 25.0;

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
    state.motion.attitude = std::clamp(options.attitude_scale, 1.0, 20.0);
    session.clock.SetSpeed(options.speed);
    session.clock.SetLoop(options.loop);
    session.Refresh();

    if (run.empty() && !session.Runs().empty()) run = session.Runs().front().data_file;  // newest
    if (!run.empty() && !state.Load(run)) {
        std::fprintf(stderr, "playback_3d: %s\n", session.Error().c_str());
    }
    state.follow_ship = options.follow;
    state.chase = options.chase;
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

void RenderScene(Renderer& renderer, const AppState& state, const ImGuiIO& io, int fb_height) {
    if (!state.scene_ready) return;
    const auto& rect = state.viewport;
    const float sx = io.DisplayFramebufferScale.x;
    const float sy = io.DisplayFramebufferScale.y;
    RenderView view;
    // OpenGL counts y from the bottom of the framebuffer
    view.viewport = {static_cast<int>(rect.pos.x * sx), fb_height - static_cast<int>((rect.pos.y + rect.size.y) * sy),
                     static_cast<int>(rect.size.x * sx), static_cast<int>(rect.size.y * sy)};
    const OrbitCamera& cam = state.camera;
    view.view = cam.View(state.scene.Origin());
    view.projection = cam.Projection();
    view.sky = kSky;
    view.fog_start = static_cast<float>(kFogStart * cam.Distance());
    view.fog_end = static_cast<float>(kFogEnd * cam.Distance());
    renderer.Render(view, state.scene);
}

bool SaveScreenshot(SDL_Window* window, const std::filesystem::path& path) {
    int w = 0;
    int h = 0;
    SDL_GL_GetDrawableSize(window, &w, &h);
    if (w <= 0 || h <= 0) return false;
    std::vector<unsigned char> pixels(static_cast<std::size_t>(w) * h * 4);
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());

    // OpenGL rows start at the bottom
    SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormat(0, w, h, 32, SDL_PIXELFORMAT_RGBA32);
    if (surface == nullptr) return false;
    for (int row = 0; row < h; row++) {
        std::copy_n(&pixels[static_cast<std::size_t>(h - 1 - row) * w * 4], w * 4,
                    static_cast<unsigned char*>(surface->pixels) + static_cast<std::size_t>(row) * surface->pitch);
    }
    const bool ok = SDL_SaveBMP(surface, path.c_str()) == 0;
    SDL_FreeSurface(surface);
    return ok;
}

SDL_Window* CreateWindow() {
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    const auto flags =
        static_cast<SDL_WindowFlags>(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);

    // Antialiased if the driver offers it
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);
    SDL_Window* window = SDL_CreateWindow("playback_3d", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                          kWindowWidth, kWindowHeight, flags);
    if (window != nullptr) return window;
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 0);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 0);
    return SDL_CreateWindow("playback_3d", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, kWindowWidth,
                            kWindowHeight, flags);
}

}  // namespace

int Run(const Options& options) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::fprintf(stderr, "playback_3d: SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }
    SDL_Window* window = CreateWindow();
    if (window == nullptr) {
        std::fprintf(stderr, "playback_3d: SDL_CreateWindow failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }
    SDL_GLContext gl = SDL_GL_CreateContext(window);
    if (gl == nullptr) {
        std::fprintf(stderr, "playback_3d: needs OpenGL 3.3: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }
    SDL_GL_MakeCurrent(window, gl);
    SDL_GL_SetSwapInterval(1);  // vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;  // layout is fixed, nothing to save
    ImGui::StyleColorsDark();
    if (options.ui_scale != 1.0f) {
        ImGui::GetStyle().ScaleAllSizes(options.ui_scale);
        io.FontGlobalScale = options.ui_scale;
    }
    ImGui_ImplSDL2_InitForOpenGL(window, gl);
    ImGui_ImplOpenGL3_Init("#version 330 core");

    int exit_code = 0;
    {
        Renderer renderer;
        std::string error;
        if (!renderer.Init(error)) {
            std::fprintf(stderr, "playback_3d: %s\n", error.c_str());
            exit_code = 1;
        }

        AppState state;
        SetupState(state, options);
        const bool screenshot_mode = !options.screenshot.empty();
        if (screenshot_mode) {
            state.session.clock.Pause();
            state.session.clock.Seek(options.screenshot_time);
            state.reset_requested = true;  // view the ship at the screenshot time
            if (options.screenshot_fit) {
                state.fit_requested = true;
                state.follow_ship = false;
            }
        }

        std::uint64_t uploaded_model = 0;
        int frame_count = 0;
        std::string title;
        Uint64 last = SDL_GetPerformanceCounter();
        bool running = exit_code == 0;
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
                state.session.Loaded() ? "playback_3d - " + state.session.GetRun().name : "playback_3d";
            if (wanted_title != title) {
                title = wanted_title;
                SDL_SetWindowTitle(window, title.c_str());
            }
            if (state.model_version != uploaded_model) {
                renderer.SetModel(state.model);
                uploaded_model = state.model_version;
            }

            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplSDL2_NewFrame();
            ImGui::NewFrame();
            DrawUi(state);
            ImGui::Render();

            int fb_w = 0;
            int fb_h = 0;
            SDL_GL_GetDrawableSize(window, &fb_w, &fb_h);
            glViewport(0, 0, fb_w, fb_h);
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            RenderScene(renderer, state, io, fb_h);
            glViewport(0, 0, fb_w, fb_h);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            // A few frames so ImGui has settled its layout before capturing
            if (screenshot_mode && ++frame_count == 3) {
                if (SaveScreenshot(window, options.screenshot)) {
                    std::printf("Saved %s\n", options.screenshot.c_str());
                } else {
                    std::fprintf(stderr, "playback_3d: screenshot failed: %s\n", SDL_GetError());
                    exit_code = 1;
                }
                running = false;
            }
            SDL_GL_SwapWindow(window);

            const Uint32 elapsed = SDL_GetTicks() - frame_start;
            if (elapsed < kMinFrameMs) SDL_Delay(kMinFrameMs - elapsed);
        }
    }  // renderer released while the context is alive

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_GL_DeleteContext(gl);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return exit_code;
}

}  // namespace playback3d
