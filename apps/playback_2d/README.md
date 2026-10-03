# playback_2d

Plays back simulation runs (`simdata/<run>/`) as a top-down 2D animation. The vessel is drawn from `vessels/<vessel>/model_2d.svg`, using the vessel named in the run's `metadata.json`, with the rudder turning with `delta_r`. You also see the track, the heading reference and the velocity vector. Playback runs in real time (1x) by default, and the speed can be changed from the GUI.

Linux only. Built with SDL2 and [Dear ImGui](https://github.com/ocornut/imgui).

### Build and run
```sh
sudo apt install libsdl2-dev        # once
cmake -B build && cmake --build build -j8
make playback                        # or ./build/playback_2d [RUN | DIR]
```
With no argument the newest run in `simdata/` is opened. RUN is a run folder, its `metadata.json` or a csv file, including csv files from before run folders. Run `./build/playback_2d --help` to list all options (`--vessel`, `--speed`, `--follow`, `--loop`, `--ui-scale`, `--screenshot`).

A run without metadata is drawn as `test_vessel`. A vessel without `model_2d.svg` is drawn as a default shape. Both are noted under Run info.

Dear ImGui is downloaded by CMake at configure time. To build offline, use `-DFETCHCONTENT_SOURCE_DIR_IMGUI=/path/to/imgui`. Without SDL2 the GUI is skipped, but the model library and its tests still build.

### Controls
| Key / mouse | Action |
|---|---|
| Space | play / pause |
| Left / Right | seek -/+ 5 s (Shift: 1 s) |
| Up / Down | faster / slower (0.1x ... 100x) |
| 1 | speed 1x |
| Home / End | start / end |
| L | loop |
| F | follow ship |
| Z, double click | fit whole track |
| Drag, wheel | pan, zoom |
| Drop a run folder or csv on the window | open it |

### Layout
```
src/
  camera.*            NED (north up, east right) <-> screen, pan/zoom/fit
  layers.*            scene layers: grid, track, reference, force, velocity, ship (model_2d.svg)
  panels.*            ImGui panels: runs, run info, telemetry, view, playback bar, viewport
  app_state.*         state shared by the panels: session, camera, vessel model
  app.*, main.cpp     SDL window, event loop, command line
tests/                unit tests for the GUI-free part (playback_2d_model)
```
Runs, csv loading, the playback clock and the shared panels come from `lib/` (`simdata`, `playback`, `playback_ui`), and the SVG loader from `vessel_model`. See [lib/README.md](../../lib/README.md). The core in `src/` is not used. The app only reads the run files.

### Extending
- **Something new to draw**: subclass `Layer` in `layers.cpp` and add it in `CreateDefaultLayers()`. It gets a checkbox and a place for its own settings in the View panel.
- **A new csv column**: add its name to `lib/simdata/channels.hpp`, plus its interpolation rule if it is an angle or a discrete value. It is then available as `frame.Get(col::kName)`. Check `frame.Has(...)` first, so older files still work.
- **A moving part on the vessel**: see `lib/vessel_model/parts.hpp`.
- **Another GUI app**: `include(${PROJECT_SOURCE_DIR}/cmake/imgui.cmake)`, then link `imgui_sdlrenderer` (2D) or `imgui_opengl3` (3D), plus `playback_ui` for the shared panels.
