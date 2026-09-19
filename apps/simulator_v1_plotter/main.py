from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import RadioButtons

DATA_DIR = Path(__file__).resolve().parents[2] / "simdata" / "simulator_v1"
MAX_FILES = 15  # only the newest files are listed

RAD2DEG = 180.0 / np.pi


def list_files():
    """Newest first, so the latest simulation is selected by default."""
    files = sorted(DATA_DIR.glob("*_simulator_v1.csv"), reverse=True)
    return files[:MAX_FILES]


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def file_label(path):
    return path.name.removesuffix("_simulator_v1.csv")


# ---------------------------------------------------------------------------
# Views. Each takes (fig, gs, d) and returns the axes it created.
# d is the loaded csv: d["t"], d["x"], ..., d["tau_N"]
# ---------------------------------------------------------------------------

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
    time_panel(ax_psi, d, ["psi"], "Heading psi [deg]", RAD2DEG)

    ax_speed = fig.add_subplot(gs[1, 1], sharex=ax_psi)
    speed = np.sqrt(d["u"] ** 2 + d["v"] ** 2 + d["w"] ** 2)
    ax_speed.plot(d["t"], speed)
    ax_speed.set_ylabel("Speed U [m/s]")
    ax_speed.set_xlabel("t [s]")
    ax_speed.grid(True)
    return [ax_ne, ax_psi, ax_speed]


def view_path(fig, gs, d):
    ax = fig.add_subplot(gs[0, 0])
    ax.plot(d["y"], d["x"])
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


# name -> (grid rows, grid cols, column width ratios, function)
VIEWS = {
    "Overview": (2, 2, [2, 1], view_overview),
    "Path": (1, 1, None, view_path),
    "Position": (3, 1, None, view_position),
    "Attitude": (3, 1, None, view_attitude),
    "Velocities": (2, 3, None, view_velocities),
    "Forces": (2, 3, None, view_forces),
}


class Plotter:
    def __init__(self, files):
        self.files = files
        self.cache = {}
        self.file = files[0]
        self.view = "Overview"
        self.plot_axes = []

        self.fig = plt.figure(figsize=(14, 8))

        ax_files = self.fig.add_axes([0.01, 0.45, 0.17, 0.50])
        ax_files.set_title("File", fontsize=10)
        self.radio_files = RadioButtons(ax_files, [file_label(f) for f in files])
        self.radio_files.on_clicked(self.select_file)

        ax_views = self.fig.add_axes([0.01, 0.10, 0.17, 0.30])
        ax_views.set_title("View", fontsize=10)
        self.radio_views = RadioButtons(ax_views, list(VIEWS))
        self.radio_views.on_clicked(self.select_view)

        for radio in (self.radio_files, self.radio_views):
            for label in radio.labels:
                label.set_fontsize(8)

        self.draw()

    def data(self):
        if self.file not in self.cache:
            self.cache[self.file] = load(self.file)
        return self.cache[self.file]

    def select_file(self, label):
        self.file = next(f for f in self.files if file_label(f) == label)
        self.draw()

    def select_view(self, label):
        self.view = label
        self.draw()

    def draw(self):
        for ax in self.plot_axes:
            ax.remove()

        d = self.data()
        rows, cols, ratios, view = VIEWS[self.view]
        gs = self.fig.add_gridspec(rows, cols, left=0.26, right=0.97, top=0.90,
                                   bottom=0.08, hspace=0.35, wspace=0.30,
                                   width_ratios=ratios)
        self.plot_axes = view(self.fig, gs, d)
        self.fig.suptitle(f"{self.view} - {self.file.name}  ({d['t'][-1]:.1f} s)")
        self.fig.canvas.draw_idle()


def main():
    print("=== simulator_v1_plotter ===")

    files = list_files()
    if not files:
        print(f"No simulations found in {DATA_DIR}")
        return

    plotter = Plotter(files)  # keep a reference so the widgets stay alive
    plt.show()


if __name__ == "__main__":
    main()
