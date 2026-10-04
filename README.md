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

### Simulator v1
Simple vessel simulator. Each run is saved as a run folder in `simdata/`. Run it with `make v1`.

### Simulator v1 plotter
<img width="1363" height="777" alt="image" src="https://github.com/user-attachments/assets/d02807a7-71e9-48e2-883d-1022b666766f" />
<img width="1363" height="777" alt="image" src="https://github.com/user-attachments/assets/f9006345-5b17-45aa-bdd5-56a2cb610634" />

### Playback 2D
Replays a run as a top-down animation, with the vessel drawn from its `model_2d.svg`. Run it with `make playback`.
<img width="1387" height="895" alt="image" src="https://github.com/user-attachments/assets/314469dc-9ef4-464b-bf20-2c3c584b41ae" />

### Playback 3D
Replays a run in 3D, with the vessel drawn from its `model_3d.obj` in its full pose (position, heave, roll, pitch, yaw). Run it with `make playback3d`.
<img width="1399" height="888" alt="image" src="https://github.com/user-attachments/assets/cc9ddda0-7221-4e7b-b39e-fe860709e9e7" />

### Tests
`make test` builds everything and runs all unit tests with CTest.
