# lib

Libraries shared by the apps. Each one is a CMake target, and headers are included as `"<library>/<header>.hpp"`. The core in `src/` is not used. These libraries only read and write files.

| Library | What it does | Dependencies |
|---|---|---|
| `simdata` | Simulation runs on disk: `RunWriter`, `metadata.json`, `ListRuns`, the csv reader (`Recording`), column names | nlohmann/json |
| `vessel_model` | `vessels/<name>/` lookup, SVG (2D) and OBJ (3D) loaders, moving parts | none |
| `playback` | `PlaybackClock` and `Session` (run list, loaded run, vessel) | simdata, vessel_model |
| `playback_ui` | ImGui widgets shared by the playback apps: run list, run info, telemetry, playback bar, keys | playback, imgui (SDL2) |
| `testing_check` | `CHECK()` for the unit tests | none |

All of them except `playback_ui` are GUI free and have unit tests in their `tests/` folder.

## Simulation runs (`simdata`)
```
simdata/YYYYMMDD_HHMMSS_<simulator>/
  metadata.json
  simulation.csv
```
`metadata.json` holds `format_version`, `simulator`, `vessel`, `created`, `dt`, `duration`, `samples`, `data_file`, `columns` (name, unit, description) and `parameters`. `parameters` is free-form, for whatever a simulator wants to keep. simulator_v1 stores `sim_time` and `guidance_modes` there. It is written last, so a folder that has only the csv is an unfinished run.

Writing a run from a new simulator:
```cpp
simdata::Metadata m;
m.simulator = "simulator_v2";
m.vessel = "test_vessel";
m.dt = dt;
m.columns = {{"t", "s", "time"}, {"x", "m", "north"}, ...};
simdata::RunWriter run(simdata::SimdataDir(simdata::FindProjectRoot()), m);
for (...) run.WriteRow({t, x, ...});   // throws if the row length does not match the columns
run.Finish();                          // writes metadata.json
```
Reading: `simdata::ListRuns(dir)` lists runs newest first, `simdata::ResolveRun(path)` accepts a run folder, its `metadata.json` or a csv, and `simdata::Recording::LoadCsv()` loads the data. Readers look up columns by name (`channels.hpp`), so new columns don't break old tools. Bump `kFormatVersion` only when old readers could misread a file.

The Python plotter follows the same rules in `apps/simulator_v1_plotter/main.py`.

## Vessels (`vessel_model`)
```
vessels/<name>/
  model_2d.svg    top view
  model_3d.obj    mesh, with model_3d.mtl for colors
```
Both models use the body frame in meters: x forward, y starboard, z down. Model each vessel at its real size, with the origin at the point the simulator's position refers to. For `test_vessel` that is midships, with z = 0 at the waterline.

- **model_2d.svg**: SVG x = body x and SVG y = body y, so a browser shows the vessel from above with the bow to the right. Paths (including curves and arcs), polygons, rects, circles, transforms and fill/stroke styles are supported. The full list is in `model_2d.hpp`.
- **model_3d.obj**: faces, normals, `o`/`g` groups and MTL colors (`Kd`, `d`). When exporting from Blender, use Forward = X and Up = -Z. Faces without normals are flat shaded, and lighting is two sided, so face winding doesn't matter.
- **Moving parts**: `parts.hpp` maps part names to csv columns. Currently `rudder` turns with `delta_r` about its leading edge, or about `data-pivot="x,y"` in the SVG. To animate something new (an azimuth thruster, for example), add one line there and give the part that name in both models.

To add a vessel, create `vessels/<name>/` with both models and set `metadata.vessel` to that name in the simulator. `test_vessel_model` checks that every vessel in the folder loads without warnings and that its 2D and 3D models are the same length. The playback apps accept `--vessel <name>` to view any run with another vessel, and they draw a default shape when a model is missing.
