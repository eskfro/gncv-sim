# Simulator v1

This app simulates a vessel and saves the run in its own folder, `simdata/YYYYMMDD_HHMMSS_simulator_v1/`:

- `metadata.json`: simulator, vessel (folder in `vessels/`), time step, duration, and every csv column with its unit and description
- `simulation.csv`: one row per time step

When it finishes, it opens the plotter on the new run. The columns are listed once, in `kColumns` in `main.cpp`. To add a column, add it there and in `snapshot_row()`, in the same order. The writer checks that the row length matches the columns.