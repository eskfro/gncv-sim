# Simulator v1 plotter

This app plots the simulation runs in `simdata/`: run folders with `metadata.json` and `simulation.csv`, and csv files from before run folders. The 15 newest runs are listed. The title shows the simulator, vessel and time step from `metadata.json`.

```sh
python3 apps/simulator_v1_plotter/main.py [RUN]
```
RUN is a run folder, its `metadata.json` or a csv file, and it is selected when the plotter opens. simulator_v1 starts the plotter this way after each run. Without RUN, the newest run is selected.
