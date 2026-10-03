# playback_3d

Plays back simulation runs (`simdata/<run>/`) as a 3D animation. The vessel is drawn from `vessels/<vessel>/model_3d.obj`, using the vessel named in the run's `metadata.json`, in its full pose: north, east, heave, roll, pitch and yaw. The rudder turns with `delta_r`. The track, time marks, heading reference, setpoint and velocity vector are drawn on the water, which is translucent, so you can see the hull below the waterline. Playback runs in real time (1x) by default, and the speed can be changed from the GUI.

Roll and pitch are often only a few degrees. Use **Roll/pitch scale** in the Camera panel (or `--exaggerate K`) to make them visible.

Linux only. Built with SDL2, OpenGL 3.3 and [Dear ImGui](https://github.com/ocornut/imgui).

### Build and run
```sh
sudo apt install libsdl2-dev libgl-dev   # once
cmake -B build && cmake --build build -j8
make playback3d                          # or ./build/playback_3d [RUN | DIR]
```
With no argument the newest run in `simdata/` is opened. RUN is a run folder, its `metadata.json` or a csv file, including csv files from before run folders. Run `./build/playback_3d --help` to list all options (`--vessel`, `--speed`, `--free`, `--chase`, `--loop`, `--exaggerate`, `--ui-scale`, `--screenshot`, `--time`, `--fit`).

A run without metadata is drawn as `test_vessel`. A vessel without `model_3d.obj` is drawn as a default box hull. Both are noted under Run info.

### Controls
| Key / mouse | Action |
|---|---|
| Space | play / pause |
| Left / Right | seek -/+ 5 s (Shift: 1 s) |
| Up / Down | faster / slower (0.1x ... 100x) |
| 1 | speed 1x |
| Home / End | start / end |
| L | loop |
| F | follow ship (camera target stays on the ship) |
| C | chase: the camera turns with the ship's heading |
| Z, double click | fit whole track |
| R | reset view: close behind the ship, follow on |
| Drag | orbit |
| Right drag, Shift drag | pan (turns follow off) |
| Wheel | zoom |
| Drop a run folder or csv on the window | open it |

### Layout
```
src/
  math3d.hpp          vectors, 4x4 matrices, R_zyx, perspective, look-at
  orbit_camera.*      camera orbiting a target on the water: yaw, pitch, distance, fit, project
  vessel_pose.*       pose from a csv frame (with roll/pitch/heave scaling), model matrices
  scene.*             per frame draw list: triangles and lines on the water, labels, vessel
  layers.*            scene layers: water, grid, track, reference, force, velocity, vessel
  renderer.*          OpenGL 3.3: lit vessel mesh, blended overlays, distance fog
  panels.*            ImGui panels: runs, run info, telemetry, camera, layers, viewport
  app_state.*         state shared by the panels: session, camera, vessel model
  app.*, main.cpp     SDL + OpenGL window, event loop, command line
tests/                unit tests for the GUI-free part (playback_3d_model)
```
Runs, csv loading, the playback clock and the shared panels come from `lib/` (`simdata`, `playback`, `playback_ui`), and the OBJ loader from `vessel_model`. See [lib/README.md](../../lib/README.md). The core in `src/` is not used. The app only reads the run files.

World coordinates are NED and body coordinates are (forward, starboard, down), as in the simulator. Positions are sent to OpenGL relative to the camera target, so large north/east values keep their precision.

### Extending
- **Something new to draw**: subclass `Layer` in `layers.cpp` and add it in `CreateDefaultLayers()`. It gets a checkbox and a place for its own settings in the Layers panel. Size things on the water in pixels with `ctx.meters_per_pixel`, so they stay the same size on screen when zooming.
- **A new csv column**: add its name to `lib/simdata/channels.hpp`, then use `frame.Get(col::kName)` in a layer. Check `frame.Has(...)` first, so older files still work.
- **A moving part on the vessel**: see `lib/vessel_model/parts.hpp`.
- **Another vessel**: add `vessels/<name>/model_3d.obj` (see [lib/README.md](../../lib/README.md)).
