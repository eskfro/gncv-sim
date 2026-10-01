# gncv-sim
Guidance, Navigation and Control of Vehicles simulator.
This is a vessel simulator made for learning the concepts and theory in TTK4190 and TTK4250.

### Dependencies
- Armadillo
- CMake
- CTest
- Python (3.14)
- SDL2 (playback-2d only: `sudo apt install libsdl2-dev`)

### Simulator v1
Simple vessel simulator. The data is saved in a csv-file.

### Simulator v1 plotter
<img width="1363" height="777" alt="image" src="https://github.com/user-attachments/assets/d02807a7-71e9-48e2-883d-1022b666766f" />
<img width="1363" height="777" alt="image" src="https://github.com/user-attachments/assets/f9006345-5b17-45aa-bdd5-56a2cb610634" />

### Playback 2D
Replays a simulator_v1 csv as a top-down animation of the vessel. Playback runs in real time by default, and the speed can be changed in the GUI. Run it with `make playback`. See [apps/playback-2d](apps/playback-2d/README.md).
