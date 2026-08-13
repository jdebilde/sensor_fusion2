from .ViewBase import ViewBase
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk


class Nav2_ekf(ViewBase):
    name = "nav2_ekf"
    default_format = "nav2_ekf"

    def __init__(self, parent):
        super().__init__(parent)

        self.fig = Figure(figsize=(9, 7), dpi=100)

        self.ax_pos = self.fig.add_subplot(221)
        self.ax_vel = self.fig.add_subplot(222)
        self.ax_acc = self.fig.add_subplot(223)
        self.ax_stop = self.fig.add_subplot(224)

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def get_series(self, field):
        return [getattr(s, field, None) for s in self.samples]

    def redraw(self):
        if not self.samples:
            return

        xs = [s.seq for s in self.samples]

        px_values = self.get_series("px")
        py_values = self.get_series("py")
        px_cm = [px * 100.0 if px is not None else None for px in px_values]
        py_cm = [py * 100.0 if py is not None else None for py in py_values]

        vx_values = self.get_series("vx")
        vy_values = self.get_series("vy")

        ax_values = self.get_series("ax")
        ay_values = self.get_series("ay")

        stopped_values = self.get_series("stopped")
        stopped01 = [self.to_stopped_value(v) for v in stopped_values]

        self.ax_pos.clear()
        self.ax_vel.clear()
        self.ax_acc.clear()
        self.ax_stop.clear()

        # 1) Position 2D
        self.ax_pos.plot(px_cm, py_cm, marker="o", markersize=2)
        self.ax_pos.set_title("Position 2D")
        self.ax_pos.set_xlabel("px [cm]")
        self.ax_pos.set_ylabel("py [cm]")
        self.ax_pos.grid(True)
        self.ax_pos.axis("equal")

        self.set_min_axis_range(self.ax_pos, px_cm, py_cm, min_range_cm=10.0)

        # Marquer le dernier point
        if px_values and py_values:
            last_px = px_values[-1]
            last_py = py_values[-1]
            if last_px is not None and last_py is not None:
                self.ax_pos.scatter([last_px], [last_py], s=40, label="current")
                self.ax_pos.legend(loc="upper right")

        # 2) Vitesse
        self.ax_vel.plot(xs, vx_values, label="vx")
        self.ax_vel.plot(xs, vy_values, label="vy")
        self.ax_vel.set_title("Vitesse")
        self.ax_vel.set_xlabel("Seq")
        self.ax_vel.set_ylabel("v")
        self.ax_vel.legend(loc="upper right")
        self.ax_vel.grid(True)

        # 3) Accélération
        self.ax_acc.plot(xs, ax_values, label="ax")
        self.ax_acc.plot(xs, ay_values, label="ay")
        self.ax_acc.set_title("Acceleration")
        self.ax_acc.set_xlabel("Seq")
        self.ax_acc.set_ylabel("a")
        self.ax_acc.legend(loc="upper right")
        self.ax_acc.grid(True)

        # 4) Stopped
        self.ax_stop.step(xs, stopped01, where="post", label="stopped")
        self.ax_stop.set_title("Etat arrêt")
        self.ax_stop.set_xlabel("Seq")
        self.ax_stop.set_ylabel("stopped")
        self.ax_stop.set_ylim(-0.1, 1.1)
        self.ax_stop.set_yticks([0, 1])
        self.ax_stop.set_yticklabels(["moving", "stopped"])
        self.ax_stop.grid(True)

        # Texte indiquant l'état courant
        current_stopped = stopped01[-1] if stopped01 else 0
        status = "STOPPED" if current_stopped == 1 else "MOVING"
        self.ax_stop.text(
            0.02,
            0.85,
            status,
            transform=self.ax_stop.transAxes,
            fontsize=12,
            bbox=dict(boxstyle="round", alpha=0.2)
        )

        self.fig.tight_layout()
        self.canvas.draw_idle()

    @staticmethod
    def to_stopped_value(value):
        return int(value) if value is not None else 0
    
    @staticmethod
    def set_min_axis_range(ax, xs, ys, min_range_cm=10.0):
        valid_x = [x for x in xs if x is not None]
        valid_y = [y for y in ys if y is not None]

        if not valid_x or not valid_y:
            ax.set_xlim(-min_range_cm / 2, min_range_cm / 2)
            ax.set_ylim(-min_range_cm / 2, min_range_cm / 2)
            return

        min_x, max_x = min(valid_x), max(valid_x)
        min_y, max_y = min(valid_y), max(valid_y)

        center_x = (min_x + max_x) / 2.0
        center_y = (min_y + max_y) / 2.0

        range_x = max_x - min_x
        range_y = max_y - min_y

        axis_range = max(range_x, range_y, min_range_cm)

        half = axis_range / 2.0

        ax.set_xlim(center_x - half, center_x + half)
        ax.set_ylim(center_y - half, center_y + half)