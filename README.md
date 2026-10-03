# gncv-sim
Guidance, Navigation and Control of Vehicles simulator.
This is a vessel simulator made for learning the concepts and theory in TTK4190 and TTK4250.

### Dependencies
- Armadillo
- CMake
- CTest
- Python (3.14)
- SDL2 (playback apps only: `sudo apt install libsdl2-dev`)
- OpenGL (playback_3d only: `sudo apt install libgl-dev`)
- nlohmann/json (optional: `sudo apt install nlohmann-json3-dev`, otherwise CMake downloads it)

### Project layout
```
src/          simulator core (dynamics, guidance, control, estimation, ...)
apps/         simulator_v1, simulator_v1_plotter, playback_2d, playback_3d
lib/          libraries shared by the apps: simdata, vessel_model, playback (see lib/README.md)
vessels/      one folder per vessel with its models (and later its physics config)
simdata/      one folder per simulation run (not in git)
```

### Vessels
Each vessel has its own folder in `vessels/`:
```
vessels/test_vessel/
  model_2d.svg    top view, used by playback_2d
  model_3d.obj    3D mesh, used by playback_3d (colors in model_3d.mtl)
```
Both models are in the body frame, in meters: x forward, y starboard, z down. `test_vessel` has the size of `vessel::VesselParams` in `src/dynamics.hpp`: 70 m long, 10 m wide, 3.25 m draft. The SVG opens in a browser or Inkscape and the OBJ opens in Blender. A part named `rudder` turns with `delta_r` from the csv. See [lib/README.md](lib/README.md) for the formats and how to add a vessel.

### Simulation data
Each run gets its own folder:
```
simdata/20261003_142501_simulator_v1/
  metadata.json   simulator, vessel, dt, duration, and every csv column with unit and description
  simulation.csv  one row per time step
```
The plotter and the playback apps list the runs newest first, and read the vessel from `metadata.json`. Csv files from before this layout (`simdata/simulator_v1/*.csv`) are still listed, as runs without metadata. `make clean-v1` deletes all simulator_v1 runs.

### Simulator v1
Simple vessel simulator. Each run is saved as a run folder in `simdata/`. Run it with `make v1`. See [apps/simulator_v1](apps/simulator_v1/README.md).

### Simulator v1 plotter
<img width="1363" height="777" alt="image" src="https://github.com/user-attachments/assets/d02807a7-71e9-48e2-883d-1022b666766f" />
<img width="1363" height="777" alt="image" src="https://github.com/user-attachments/assets/f9006345-5b17-45aa-bdd5-56a2cb610634" />

### Playback 2D
Replays a run as a top-down animation, with the vessel drawn from its `model_2d.svg`. Playback runs in real time by default, and the speed can be changed in the GUI. Run it with `make playback`. See [apps/playback_2d](apps/playback_2d/README.md).

### Playback 3D
Replays a run in 3D, with the vessel drawn from its `model_3d.obj` in its full pose (position, heave, roll, pitch, yaw) and the rudder moving. The track, references and velocity are drawn on the water. Roll and pitch can be exaggerated to make them visible. Run it with `make playback3d`. See [apps/playback_3d](apps/playback_3d/README.md).

### Tests
`make test` builds everything and runs all unit tests with CTest.
