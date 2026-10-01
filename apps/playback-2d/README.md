# playback-2d

Plays back simulator_v1 recordings (`simdata/simulator_v1/*.csv`) as a top-down 2D animation. You see the vessel hull with its rudder, the track, the heading reference and the velocity vector. Playback runs in real time (1x) by default, and the speed can be changed from the GUI.

Linux only. Built with SDL2 and [Dear ImGui](https://github.com/ocornut/imgui).

### Build and run
```sh
sudo apt install libsdl2-dev        # once
cmake -B build && cmake --build build -j8
make playback                        # or ./build/playback-2d [FILE.csv | DIR]
```
With no argument the newest csv in `simdata/simulator_v1` is opened. Run `./build/playback-2d --help` to list all options (`--speed`, `--follow`, `--loop`, `--ui-scale`, `--screenshot`).

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
| Drop a csv on the window | open it |

### Layout
```
src/
  channels.hpp        csv column names and interpolation rules (simulator_v1 format)
  recording.*         csv loader, columns by name, interpolated Frame at any time
  playback_clock.*    wall clock -> simulation time, speed, loop
  camera.*            NED (north up, east right) <-> screen, pan/zoom/fit
  data_files.*        finding simdata/simulator_v1 and its csv files
  layers.*            scene layers: grid, track, reference, force, velocity, ship
  panels.*            ImGui panels: files, telemetry, view, playback bar, viewport
  app_state.*         state shared by the panels
  app.*, main.cpp     SDL window, event loop, command line
tests/                unit tests for the GUI-free part (playback-2d-model)
```
The core in `src/` is not used. The app only reads the csv files.

### Extending
- **Something new to draw**: subclass `Layer` in `layers.cpp` and add it in `CreateDefaultLayers()`. It gets a checkbox and a place for its own settings in the View panel.
- **A new csv column**: add its name to `channels.hpp`, plus its interpolation rule if it is an angle or a discrete value. It is then available as `frame.Get(col::kName)`. Check `frame.Has(...)` first, so older files still work.
- **Another GUI app**: `include(${PROJECT_SOURCE_DIR}/cmake/imgui.cmake)` and link `imgui`.
