"""Plots simulation runs from simdata/.

Usage: python3 main.py [RUN]

RUN is a run folder (simdata/<run>/), its metadata.json or a csv file.
Without it the newest run is selected.
"""
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import RadioButtons

SIMDATA_DIR = Path(__file__).resolve().parents[2] / "simdata"
METADATA_FILE = "metadata.json"
DATA_FILE = "simulation.csv"
MAX_RUNS = 15  # only the newest runs are listed

RAD2DEG = 180.0 / np.pi

# Same order as the GuidanceMode enum in common.hpp
GUIDANCE_MODES = ["HeadingHold", "PositionHold", "WaypointTracking"]

# Columns added after the first csv version. Older files lack them.
REF_COLS = ["guidance_mode", "x_d", "y_d", "psi_d", "u_d"]
ACT_COLS = ["delta_r_ref", "delta_r_cmd", "delta_r",
            "n_mp_ref", "n_mp_cmd", "n_mp", "n_tt_ref", "n_tt_cmd", "n_tt"]


# ---------------------------------------------------------------------------
# Runs on disk. Each simulation has its own folder:
#   simdata/YYYYMMDD_HHMMSS_simulator_v1/metadata.json + simulation.csv
# Loose csv files from before (simdata/simulator_v1/*.csv) are listed too.
# Same rules as lib/simdata/run.hpp.
# ---------------------------------------------------------------------------

@dataclass
class Run:
    name: str
    csv: Path
    metadata: dict  # empty when the run has no metadata.json

    @property
    def sort_key(self):
        # Legacy names have HH:MM:SS, new ones HHMMSS
        return self.name.replace(":", "")


def read_metadata(run_dir):
    try:
        with open(run_dir / METADATA_FILE) as f:
            return json.load(f)
    except (OSError, ValueError) as e:
        print(f"Ignoring {run_dir / METADATA_FILE}: {e}")
        return {}


def resolve_run(path):
    """The run a path refers to: a run folder, its metadata.json or a csv file. None otherwise."""
    path = Path(path)
    if path.is_dir():
        if not ((path / METADATA_FILE).is_file() or (path / DATA_FILE).is_file()):
            return None
        metadata = read_metadata(path) if (path / METADATA_FILE).is_file() else {}
        csv = path / Path(metadata.get("data_file") or DATA_FILE).name
        return Run(path.name, csv, metadata)
    if path.name == METADATA_FILE:
        return resolve_run(path.parent)
    if path.suffix == ".csv" and path.is_file():
        if (path.parent / METADATA_FILE).is_file():
            return Run(path.parent.name, path, read_metadata(path.parent))
        return Run(path.stem, path, {})
    return None


def list_runs():
    """Newest first, so the latest simulation is selected by default."""
    runs = []
    if SIMDATA_DIR.is_dir():
        for entry in SIMDATA_DIR.iterdir():
            if entry.suffix == ".csv" and entry.is_file():
                runs.append(Run(entry.stem, entry, {}))
            elif entry.is_dir():
                run = resolve_run(entry)
                if run:
                    runs.append(run)
                else:  # a folder of legacy csv files
                    runs += [Run(f.stem, f, {}) for f in entry.glob("*.csv")]
    runs.sort(key=lambda r: r.sort_key, reverse=True)
    return runs[:MAX_RUNS]


def load(run):
    return np.genfromtxt(run.csv, delimiter=",", names=True)


def run_label(run):
    return run.name.removesuffix("_simulator_v1")


def run_title(run):
    """Run name plus what metadata.json says about it."""
    m = run.metadata
    details = [m[k] for k in ("simulator", "vessel") if m.get(k)]
    if m.get("dt"):
        details.append(f"dt = {m['dt']:g} s")
    return f"{run.name}" + (f"  [{', '.join(details)}]" if details else "  [no metadata]")


# ---------------------------------------------------------------------------
# Views. Each takes (fig, gs, d) and returns the axes it created.
# d is the loaded csv: d["t"], d["x"], ..., d["tau_N"]
# ---------------------------------------------------------------------------

def has(d, cols):
    return all(c in d.dtype.names for c in cols)


def wrap(angle):
    """Smallest signed angle, in (-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def ref_panel(ax, d, series, ylabel, scale=1.0):
    """series: list of (column, label, plot style). Reference dashed, state solid."""
    for col, label, style in series:
        ax.plot(d["t"], d[col] * scale, style, label=label)
    ax.set_ylabel(ylabel)
    ax.grid(True)
    ax.legend(loc="upper right", fontsize=8)


def time_panel(ax, d, cols, ylabel, scale=1.0):
    for col in cols:
        ax.plot(d["t"], d[col] * scale, label=col)
    ax.set_ylabel(ylabel)
    ax.grid(True)
    if len(cols) > 1:
        ax.legend(loc="upper right")


def view_overview(fig, gs, d):
    ax_ne = fig.add_subplot(gs[:, 0])
    ax_ne.plot(d["y"], d["x"])
    ax_ne.plot(d["y"][0], d["x"][0], "go", label="start")
    ax_ne.plot(d["y"][-1], d["x"][-1], "rs", label="end")
    ax_ne.set_title("Path (NED)")
    ax_ne.set_xlabel("East y [m]")
    ax_ne.set_ylabel("North x [m]")
    ax_ne.set_aspect("equal", adjustable="datalim")  # 1 m east = 1 m north
    ax_ne.grid(True)
    ax_ne.legend()

    ax_psi = fig.add_subplot(gs[0, 1])
    if has(d, ["psi_d"]):
        ref_panel(ax_psi, d, [("psi", "psi", "C0-"), ("psi_d", "psi_d", "k--")],
                  "Heading psi [deg]", RAD2DEG)
    else:
        time_panel(ax_psi, d, ["psi"], "Heading psi [deg]", RAD2DEG)

    ax_speed = fig.add_subplot(gs[1, 1], sharex=ax_psi)
    speed = np.sqrt(d["u"] ** 2 + d["v"] ** 2 + d["w"] ** 2)
    ax_speed.plot(d["t"], speed, label="U")
    if has(d, ["u_d"]):
        ax_speed.plot(d["t"], d["u_d"], "k--", label="u_d")
        ax_speed.legend(loc="upper right", fontsize=8)
    ax_speed.set_ylabel("Speed U [m/s]")
    ax_speed.set_xlabel("t [s]")
    ax_speed.grid(True)
    return [ax_ne, ax_psi, ax_speed]


def view_path(fig, gs, d):
    ax = fig.add_subplot(gs[0, 0])
    ax.plot(d["y"], d["x"], label="actual")
    # x_d, y_d are all zero in HeadingHold, so only draw them when used
    if has(d, ["x_d", "y_d"]) and (np.any(d["x_d"]) or np.any(d["y_d"])):
        ax.plot(d["y_d"], d["x_d"], "k--", label="reference")
    ax.plot(d["y"][0], d["x"][0], "go", label="start")
    ax.plot(d["y"][-1], d["x"][-1], "rs", label="end")

    # Time markers every 20 s
    for t in np.arange(20, d["t"][-1], 20):
        i = np.searchsorted(d["t"], t)
        ax.plot(d["y"][i], d["x"][i], "k.")
        ax.annotate(f"{t:.0f} s", (d["y"][i], d["x"][i]), xytext=(5, 5),
                    textcoords="offset points", fontsize=8)

    ax.set_title("Path (NED), 1:1")
    ax.set_xlabel("East y [m]")
    ax.set_ylabel("North x [m]")
    ax.set_aspect("equal", adjustable="datalim")  # 1 m east = 1 m north
    ax.grid(True)
    ax.legend()
    return [ax]


def view_position(fig, gs, d):
    axes = []
    for i, (col, label) in enumerate([("x", "North x [m]"),
                                      ("y", "East y [m]"),
                                      ("z", "Down z [m]")]):
        ax = fig.add_subplot(gs[i, 0], sharex=axes[0] if axes else None)
        time_panel(ax, d, [col], label)
        axes.append(ax)
    axes[-1].set_xlabel("t [s]")
    return axes


def view_attitude(fig, gs, d):
    axes = []
    for i, (col, label) in enumerate([("phi", "Roll phi [deg]"),
                                      ("theta", "Pitch theta [deg]"),
                                      ("psi", "Yaw psi [deg]")]):
        ax = fig.add_subplot(gs[i, 0], sharex=axes[0] if axes else None)
        time_panel(ax, d, [col], label, RAD2DEG)
        axes.append(ax)
    axes[-1].set_xlabel("t [s]")
    return axes


def view_velocities(fig, gs, d):
    panels = [
        ("u", "Surge u [m/s]", 1.0), ("v", "Sway v [m/s]", 1.0), ("w", "Heave w [m/s]", 1.0),
        ("p", "Roll rate p [deg/s]", RAD2DEG), ("q", "Pitch rate q [deg/s]", RAD2DEG),
        ("r", "Yaw rate r [deg/s]", RAD2DEG),
    ]
    axes = []
    for i, (col, label, scale) in enumerate(panels):
        ax = fig.add_subplot(gs[i // 3, i % 3], sharex=axes[0] if axes else None)
        time_panel(ax, d, [col], label, scale)
        if i >= 3:
            ax.set_xlabel("t [s]")
        axes.append(ax)
    return axes


def view_forces(fig, gs, d):
    panels = [
        ("tau_X", "Surge force X [N]"), ("tau_Y", "Sway force Y [N]"),
        ("tau_Z", "Heave force Z [N]"), ("tau_K", "Roll moment K [Nm]"),
        ("tau_M", "Pitch moment M [Nm]"), ("tau_N", "Yaw moment N [Nm]"),
    ]
    axes = []
    for i, (col, label) in enumerate(panels):
        ax = fig.add_subplot(gs[i // 3, i % 3], sharex=axes[0] if axes else None)
        time_panel(ax, d, [col], label)
        if i >= 3:
            ax.set_xlabel("t [s]")
        axes.append(ax)
    return axes


def view_tracking(fig, gs, d):
    """Reference vs actual, the errors, and which guidance mode is active."""
    ax_psi = fig.add_subplot(gs[0, 0])
    ref_panel(ax_psi, d, [("psi", "psi", "C0-"), ("psi_d", "psi_d", "k--")],
              "Heading [deg]", RAD2DEG)
    ax_psi.set_title("Heading")

    ax_psi_err = fig.add_subplot(gs[0, 1], sharex=ax_psi)
    ax_psi_err.plot(d["t"], wrap(d["psi_d"] - d["psi"]) * RAD2DEG, "C3")
    ax_psi_err.set_ylabel("psi_d - psi [deg]")
    ax_psi_err.set_title("Heading error")
    ax_psi_err.grid(True)

    ax_u = fig.add_subplot(gs[1, 0], sharex=ax_psi)
    ref_panel(ax_u, d, [("u", "u", "C0-"), ("u_d", "u_d", "k--")], "Surge u [m/s]")

    ax_u_err = fig.add_subplot(gs[1, 1], sharex=ax_psi)
    ax_u_err.plot(d["t"], d["u_d"] - d["u"], "C3")
    ax_u_err.set_ylabel("u_d - u [m/s]")
    ax_u_err.grid(True)

    ax_mode = fig.add_subplot(gs[2, :], sharex=ax_psi)
    ax_mode.step(d["t"], d["guidance_mode"], where="post", color="C2")
    ax_mode.set_yticks(range(len(GUIDANCE_MODES)))
    ax_mode.set_yticklabels(GUIDANCE_MODES, fontsize=8)
    ax_mode.set_ylim(-0.5, len(GUIDANCE_MODES) - 0.5)
    ax_mode.set_ylabel("Mode")
    ax_mode.set_xlabel("t [s]")
    ax_mode.grid(True)
    return [ax_psi, ax_psi_err, ax_u, ax_u_err, ax_mode]


def view_actuators(fig, gs, d):
    """Reference -> command -> state for each actuator (rudder in deg, propellers in rpm)."""
    panels = [
        ("delta_r", "Rudder delta_r [deg]", RAD2DEG),
        ("n_mp", "Main propulsor n_mp [rpm]", 1.0),
        ("n_tt", "Tunnel thruster n_tt [rpm]", 1.0),
    ]
    axes = []
    for i, (name, label, scale) in enumerate(panels):
        ax = fig.add_subplot(gs[i, 0], sharex=axes[0] if axes else None)
        ref_panel(ax, d, [(f"{name}_ref", "reference", "k--"),
                          (f"{name}_cmd", "command", "C1-"),
                          (name, "state", "C0-")], label, scale)
        axes.append(ax)
    axes[-1].set_xlabel("t [s]")
    return axes


def view_actuator_forces(fig, gs, d):
    """What the actuators produce: total tau next to the states that drive it."""
    ax_tau = fig.add_subplot(gs[0, 0])
    time_panel(ax_tau, d, ["tau_X", "tau_Y"], "Force [N]")
    ax_tau.set_title("Forces")

    ax_n = fig.add_subplot(gs[1, 0], sharex=ax_tau)
    time_panel(ax_n, d, ["tau_N"], "Yaw moment N [Nm]")

    ax_act = fig.add_subplot(gs[0, 1], sharex=ax_tau)
    ax_act.plot(d["t"], d["n_mp"], label="n_mp")
    ax_act.plot(d["t"], d["n_tt"], label="n_tt")
    ax_act.set_ylabel("rpm")
    ax_act.set_title("Propeller rpm")
    ax_act.grid(True)
    ax_act.legend(loc="upper right", fontsize=8)

    ax_rud = fig.add_subplot(gs[1, 1], sharex=ax_tau)
    ax_rud.plot(d["t"], d["delta_r"] * RAD2DEG)
    ax_rud.set_ylabel("Rudder delta_r [deg]")
    ax_rud.grid(True)

    for ax in (ax_n, ax_rud):
        ax.set_xlabel("t [s]")
    return [ax_tau, ax_n, ax_act, ax_rud]


# name -> (grid rows, grid cols, column width ratios, row height ratios, function)
VIEWS = {
    "Overview": (2, 2, [2, 1], None, view_overview),
    "Path": (1, 1, None, None, view_path),
    "Tracking": (3, 2, None, [3, 3, 1], view_tracking),
    "Position": (3, 1, None, None, view_position),
    "Attitude": (3, 1, None, None, view_attitude),
    "Velocities": (2, 3, None, None, view_velocities),
    "Forces": (2, 3, None, None, view_forces),
    "Actuators": (3, 1, None, None, view_actuators),
    "Act. & forces": (2, 2, None, None, view_actuator_forces),
}

# Views that need the columns added after the first csv version
REQUIRES = {
    "Tracking": REF_COLS,
    "Actuators": ACT_COLS,
    "Act. & forces": ACT_COLS,
}


class Plotter:
    def __init__(self, runs, selected=0):
        self.runs = runs
        self.cache = {}
        self.run = runs[selected]
        self.view = "Overview"
        self.plot_axes = []

        self.fig = plt.figure(figsize=(14, 8))

        ax_runs = self.fig.add_axes([0.01, 0.45, 0.17, 0.50])
        ax_runs.set_title("Run", fontsize=10)
        self.radio_runs = RadioButtons(ax_runs, [run_label(r) for r in runs], active=selected)
        self.radio_runs.on_clicked(self.select_run)

        ax_views = self.fig.add_axes([0.01, 0.10, 0.17, 0.30])
        ax_views.set_title("View", fontsize=10)
        self.radio_views = RadioButtons(ax_views, list(VIEWS))
        self.radio_views.on_clicked(self.select_view)

        for radio in (self.radio_runs, self.radio_views):
            for label in radio.labels:
                label.set_fontsize(8)

        self.draw()

    def data(self):
        if self.run.csv not in self.cache:
            self.cache[self.run.csv] = load(self.run)
        return self.cache[self.run.csv]

    def select_run(self, label):
        self.run = next(r for r in self.runs if run_label(r) == label)
        self.draw()

    def select_view(self, label):
        self.view = label
        self.draw()

    def draw(self):
        for ax in self.plot_axes:
            ax.remove()

        d = self.data()
        rows, cols, width_ratios, height_ratios, view = VIEWS[self.view]

        missing = [c for c in REQUIRES.get(self.view, []) if c not in d.dtype.names]
        if missing:
            # Old csv without the new columns: say so instead of raising KeyError
            ax = self.fig.add_axes([0.26, 0.08, 0.71, 0.82])
            ax.axis("off")
            ax.text(0.5, 0.5, f"{self.run.name}\nhas no data for this view\n"
                              f"(missing: {', '.join(missing[:3])}, ...)",
                    ha="center", va="center")
            self.plot_axes = [ax]
        else:
            gs = self.fig.add_gridspec(rows, cols, left=0.26, right=0.97, top=0.90,
                                       bottom=0.08, hspace=0.35, wspace=0.30,
                                       width_ratios=width_ratios,
                                       height_ratios=height_ratios)
            self.plot_axes = view(self.fig, gs, d)
        self.fig.suptitle(f"{self.view} - {run_title(self.run)}  ({d['t'][-1]:.1f} s)")
        self.fig.canvas.draw_idle()


def main():
    print("=== simulator_v1_plotter ===")

    runs = list_runs()
    selected = 0
    if len(sys.argv) > 1:
        run = resolve_run(sys.argv[1])
        if run is None:
            print(f"{sys.argv[1]} is not a simulation run (a run folder, metadata.json or csv file)")
            return
        same = [i for i, r in enumerate(runs) if r.csv.resolve() == run.csv.resolve()]
        if same:
            selected = same[0]
        else:
            runs.insert(0, run)
    if not runs:
        print(f"No simulations found in {SIMDATA_DIR}")
        return

    plotter = Plotter(runs, selected)  # keep a reference so the widgets stay alive
    plt.show()


if __name__ == "__main__":
    main()
